#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "u8g2_esp32_hal.h"

/**
 * Handle actual spi/i2c implementation for u8g2.
 * This is very much setup to work with just esp32 and max7219 (just spi).
 * Since the esp32 api requires you to provide the data of one transaction at once,
 * but the max7219 u8g2 code gives the data byte for byte, we need to add some buffering.
 * The only (other) implementation of this I found online is:
 * https://github.com/nodemcu/nodemcu-firmware/blob/master/app/platform/u8x8_nodemcu_hal.c
 */
static spi_device_handle_t handle_spi;      // SPI handle.
static u8g2_esp32_hal_t    u8g2_esp32_hal;  // HAL state data.

/*
 * Initialze the ESP32 HAL.
 */
void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param) {
	u8g2_esp32_hal = u8g2_esp32_hal_param;
} // u8g2_esp32_hal_init

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle SPI communications.
 */
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {

	// Should be big enough right?
	static uint8_t buffer[64];
	static uint8_t buf_idx;
	uint8_t *data;

	switch(msg) {
		case U8X8_MSG_BYTE_INIT: {
			if (u8g2_esp32_hal.clk == U8G2_ESP32_HAL_UNDEFINED ||
					u8g2_esp32_hal.mosi == U8G2_ESP32_HAL_UNDEFINED ||
					u8g2_esp32_hal.cs == U8G2_ESP32_HAL_UNDEFINED) {
				break;
			}

		  spi_bus_config_t bus_config = {};
		  bus_config.sclk_io_num   = u8g2_esp32_hal.clk; // CLK
		  bus_config.mosi_io_num   = u8g2_esp32_hal.mosi; // MOSI
		  bus_config.miso_io_num   = -1; // MISO
		  bus_config.quadwp_io_num = -1; // Not used
		  bus_config.quadhd_io_num = -1; // Not used
		  ESP_ERROR_CHECK(spi_bus_initialize(u8g2_esp32_hal.host, &bus_config, SPI_DMA_CH1));

		  spi_device_interface_config_t dev_config = {};
		  dev_config.mode             = 0;
		  dev_config.clock_speed_hz   = u8g2_esp32_hal.clock_speed_hz;
		  dev_config.spics_io_num     = u8g2_esp32_hal.cs;
		  dev_config.flags            = SPI_DEVICE_NO_DUMMY;
		  dev_config.queue_size       = 1;
		  ESP_ERROR_CHECK(spi_bus_add_device(u8g2_esp32_hal.host, &dev_config, &handle_spi));

		  break;
		}

		case U8X8_MSG_BYTE_SEND:
		    data = (uint8_t *)arg_ptr;
		    while( arg_int > 0 ) {
				assert(buf_idx < sizeof(buffer) / sizeof(buffer[0]));
		    	buffer[buf_idx++] = *data;
		    	data++;
		    	arg_int--;
		    }
			break;

		case U8X8_MSG_BYTE_START_TRANSFER:
			buf_idx = 0;
			break;

		case U8X8_MSG_BYTE_END_TRANSFER:
			spi_transaction_t trans_desc = {};
			trans_desc.length    = 8 * buf_idx; // Number of bits NOT number of bytes.
			trans_desc.tx_buffer = (void *)buffer;
			trans_desc.rx_buffer = NULL;

			ESP_ERROR_CHECK(spi_device_transmit(handle_spi, &trans_desc));
			break;
	}
	return 0;
} // u8g2_esp32_spi_byte_cb

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle callbacks for GPIO and delay functions.
 */
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	switch(msg) {
		// Delay for the number of milliseconds passed in through arg_int.
		case U8X8_MSG_DELAY_MILLI:
			vTaskDelay(arg_int/portTICK_PERIOD_MS);
			break;
	}
	return 0;
} // u8g2_esp32_gpio_and_delay_cb
