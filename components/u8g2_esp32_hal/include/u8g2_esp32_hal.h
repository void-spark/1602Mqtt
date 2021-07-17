#ifndef U8G2_ESP32_HAL_H_
#define U8G2_ESP32_HAL_H_
#include "u8g2.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#define U8G2_ESP32_HAL_UNDEFINED (-1)

typedef struct {
	spi_host_device_t host;
	int clock_speed_hz;
	int clk;
	int mosi;
	int cs;
	int reset;
	int dc;
} u8g2_esp32_hal_t ;

#define U8G2_ESP32_HAL_DEFAULT {HSPI_HOST, SPI_MASTER_FREQ_8M, U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED }

void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param);
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
#endif /* U8G2_ESP32_HAL_H_ */
