/*
Copyright 2017 Knut Auvor Grythe

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "bme280_twi.h"
#include "sdk_errors.h"
#include "app_error.h"

#define _REG_CTRL_HUM  0xF2U
#define _REG_CTRL_MEAS 0xF4U
#define _REG_CONFIG    0xF5U
#define _REG_TEMP_MSB  0xFAU
#define _REG_TEMP_LSB  0xFBU
#define _REG_TEMP_XLSB 0xFCU

#define _REG_PRES_MSB  0xF7U
#define _REG_PRES_LSB  0xF8U
#define _REG_PRES_XLSB 0xF9U

#define _REG_DIG_T1_LSB 0x88U
#define _REG_DIG_T1_MSB 0x89U
#define _REG_DIG_T2_LSB 0x8AU
#define _REG_DIG_T2_MSB 0x8BU
#define _REG_DIG_T3_LSB 0x8CU
#define _REG_DIG_T3_MSB 0x8DU

#define _REG_DIG_P1_LSB 0x8EU
#define _REG_DIG_P1_MSB 0x8FU
#define _REG_DIG_P2_LSB 0x90U
#define _REG_DIG_P2_MSB 0x91U
#define _REG_DIG_P3_LSB 0x92U
#define _REG_DIG_P3_MSB 0x93U

#define _SHIFT_OSRS_T 0x05U
#define _SHIFT_OSRS_P 0x02U
#define _SHIFT_T_SB   0x05U
#define _SHIFT_FILTER 0x02U

static volatile bool    m_xfer_done = false;
static volatile uint8_t m_rx_reg = 0;
static int32_t  m_temp_raw;
static int32_t  m_pres_raw;

static const nrf_drv_twi_t *m_twi_ptr;
static bme280_twi_evt_handler_t m_handler;
static uint8_t	m_addr;
static void *   m_context;
static uint8_t  m_buf[20];
static uint16_t m_dig_T1;
static int16_t  m_dig_T2;
static int16_t  m_dig_T3;
static uint16_t m_dig_P1;
static int16_t  m_dig_P2;
static int16_t  m_dig_P3;
static int16_t  m_dig_P4;
static int16_t  m_dig_P5;
static int16_t  m_dig_P6;
static int16_t  m_dig_P7;
static int16_t  m_dig_P8;
static int16_t  m_dig_P9;
static int32_t  m_t_fine = 0;
static uint8_t  m_ctrl_meas;

__STATIC_INLINE int32_t _merge_20_bit(uint8_t *buf) {
	return (((uint32_t)buf[0]) << 12 | ((uint32_t)buf[1]) << 4 | buf[0] >> 4);
}

__STATIC_INLINE int16_t _merge_16_bit(uint8_t *buf) {
	return (((uint16_t)buf[0]) << 8 | buf[1]);
}

__STATIC_INLINE void _send_event(bme280_twi_evt_type_t event_type)
{
	bme280_twi_evt_t event = {
		.type = event_type
	};
	m_handler(&event, m_context);
}

static void _compensate_temp(int32_t adc_T, int32_t *temp)
{
	int32_t var1 = ((((adc_T>>3) - ((int32_t)m_dig_T1<<1))) * ((int32_t)m_dig_T2)) >> 11;
	int32_t var2 = (((((adc_T>>4) - ((int32_t)m_dig_T1)) * ((adc_T>>4) - ((int32_t)m_dig_T1))) >> 12) *
			((int32_t)m_dig_T3)) >> 14;
	m_t_fine = var1 + var2;
	*temp = ((var1 + var2) * 5 + 128) >> 8;
}


static void _compensate_pres(int32_t adc_P, uint32_t *pres)
{//based on zefyr https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/sensor/bme280/bme280.c
	int64_t var1, var2, p;

	var1 = ((int64_t)m_t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)m_dig_P6;
	var2 = var2 + ((var1 * (int64_t)m_dig_P5) << 17);
	var2 = var2 + (((int64_t)m_dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)m_dig_P3) >> 8) +
		((var1 * (int64_t)m_dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)m_dig_P1) >> 33;

	/* Avoid exception caused by division by zero. */
	if (var1 == 0) {
		*pres = 0U;
		return;
	}

	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)m_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)m_dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)m_dig_P7) << 4);
	*pres = (uint32_t)p;
}

static void _write(uint8_t reg, uint8_t data) {
	m_xfer_done = false;
	uint8_t buf[2] = {reg, data};
	ret_code_t err_code = nrf_drv_twi_tx(m_twi_ptr, m_addr, buf, sizeof(buf), false);
	APP_ERROR_CHECK(err_code);
}

static void _write_blocking(uint8_t reg, uint8_t data) {
	_write(reg, data);
	do {
		__WFE();
	} while (m_xfer_done == false);
}

static void _read(uint8_t reg, uint8_t *buf, int len) {
	m_xfer_done = false;
	m_rx_reg = reg;
	nrf_drv_twi_xfer_desc_t desc = {
		.type = NRF_DRV_TWI_XFER_TXRX,
		.address = m_addr,
		.primary_length = sizeof(reg),
		.secondary_length = sizeof(*buf) * len,
		.p_primary_buf = &reg,
		.p_secondary_buf = buf
	};
	ret_code_t err_code = nrf_drv_twi_xfer(m_twi_ptr, &desc, 0);
	APP_ERROR_CHECK(err_code);
}

static void _read_blocking(uint8_t reg, uint8_t *buf, int len) {
	_read(reg, buf, len);
	do {
		__WFE();
	} while (m_xfer_done == false);
}

void bme280_twi_evt_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
	switch (p_event->type) {
		case NRF_DRV_TWI_EVT_DONE:
			if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TXRX) {
				switch (m_rx_reg) {
					case _REG_TEMP_MSB:
						m_temp_raw = _merge_20_bit(m_buf);
						_send_event(BME280_TWI_MEASUREMENT_FETCHED);
						break;
					case _REG_PRES_MSB:
						m_pres_raw = _merge_20_bit(m_buf);
						_send_event(BME280_TWI_MEASUREMENT_FETCHED);
						break;
				}
				m_rx_reg = 0;
			}
			m_xfer_done = true;
			break;
		default:
			break;
	}
}

void bme280_twi_init(nrf_drv_twi_t const *       p_twi,
                     bme280_twi_config_t const * p_config,
                     bme280_twi_evt_handler_t    event_handler,
                     void *                      p_context)
{
	m_twi_ptr = p_twi;
	m_context = p_context;
	m_handler = event_handler;
	m_addr = p_config->addr;

	_read_blocking(_REG_DIG_T1_MSB, m_buf, 6);
	m_dig_T1 = _merge_16_bit(m_buf);
	m_dig_T2 = _merge_16_bit(m_buf + 2);
	m_dig_T3 = _merge_16_bit(m_buf + 4);
	
	_read_blocking(_REG_DIG_P1_MSB, m_buf, 18);
	m_dig_P1 = _merge_16_bit(m_buf);
	m_dig_P2 = _merge_16_bit(m_buf + 2);
	m_dig_P3 = _merge_16_bit(m_buf + 4);
	m_dig_P4 = _merge_16_bit(m_buf + 6);
	m_dig_P5 = _merge_16_bit(m_buf + 8);
	m_dig_P6 = _merge_16_bit(m_buf + 10);
	m_dig_P7 = _merge_16_bit(m_buf + 12);
	m_dig_P8 = _merge_16_bit(m_buf + 14);
	m_dig_P9 = _merge_16_bit(m_buf + 16);

	// Write CONFIG first, because it is only guaranteed to take effect in sleep mode.
	_write_blocking(_REG_CONFIG, ((p_config->standby << _SHIFT_T_SB) | (p_config->filter << _SHIFT_FILTER)));

	// Write CTRL_HUM next, because it only takes effect after writing CTRL_MEAS.
	_write_blocking(_REG_CTRL_HUM, BME280_TWI_OVERSAMPLING_SKIPPED);

	// Calculate, but don't write CTRL_MEAS yet. It will be written by bme280_twi_enable.
	m_ctrl_meas = ((p_config->temp_oversampling << _SHIFT_OSRS_T)
			| (p_config->pres_oversampling << _SHIFT_OSRS_P)
			| BME280_TWI_MODE_FORCED);
}

void bme280_twi_enable(void)
{
	// Write CTRL_MEAS to start the show
	_write_blocking(_REG_CTRL_MEAS, m_ctrl_meas);
}

void bme280_twi_measurement_fetch(void)
{
	_read_blocking(_REG_TEMP_MSB, m_buf, 3);
	_read_blocking(_REG_PRES_MSB, m_buf, 3);
}

void bme280_twi_measurement_get(bme280_twi_data_t *data)
{
	_compensate_temp(m_temp_raw, &data->temp);
	_compensate_pres(m_pres_raw, &data->pres);
	
}
