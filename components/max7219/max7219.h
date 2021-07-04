#ifndef __MAX7219_H__
#define __MAX7219_H__

#include <stdint.h>
#include <stdbool.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX7219_MAX_CASCADE_SIZE 8
#define MAX7219_MAX_BRIGHTNESS   15

/**
 * Display descriptor
 */
typedef struct
{
    spi_device_interface_config_t spi_cfg;
    spi_device_handle_t spi_dev;
    uint8_t cascade_size;        //!< Up to `MAX7219_MAX_CASCADE_SIZE` MAX721xx cascaded
    bool mirrored;               //!< true for horizontally mirrored displays
} max7219_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param host SPI host
 * @param cs_pin CS GPIO number
 * @return `ESP_OK` on success
 */
esp_err_t max7219_init_desc(max7219_t *dev, spi_host_device_t host, gpio_num_t cs_pin);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max7219_free_desc(max7219_t *dev);

/**
 * @brief Initialize display
 *
 * Switch it to normal operation from shutdown mode,
 * set scan limit to the max and clear
 *
 * @param dev Display descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max7219_init(max7219_t *dev);

/**
 * @brief Set display brightness
 *
 * @param dev Display descriptor
 * @param value Brightness value, 0..MAX7219_MAX_BRIGHTNESS
 * @return `ESP_OK` on success
 */
esp_err_t max7219_set_brightness(max7219_t *dev, uint8_t value);

/**
 * @brief Shutdown display or set it to normal mode
 *
 * @param dev Display descriptor
 * @param shutdown Shutdown display if true
 * @return `ESP_OK` on success
 */
esp_err_t max7219_set_shutdown_mode(max7219_t *dev, bool shutdown);

/**
 * @brief Clear display
 *
 * @param dev Display descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max7219_clear(max7219_t *dev);

/**
 * @brief Draw 64-bit image(s) on 8x8 matrix(s)
 *
 * @param dev Display descriptor
 * @param count Count of 8x8 images
 * @param image 64-bit buffer with image data
 * @return `ESP_OK` on success
 */
esp_err_t max7219_draw_images_8x8(max7219_t *dev, uint8_t count, const void *image);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MAX7219_H__ */
