#include "max7219.h"
#include <string.h>
#include <esp_log.h>

static const char *TAG = "max7219";

#define CLOCK_SPEED_HZ (10000000) // 10 MHz

#define ALL_CHIPS 0xff
#define ALL_DIGITS 8

#define REG_DIGIT_0      (0x1 << 8)
#define REG_DECODE_MODE  (0x9 << 8)
#define REG_INTENSITY    (0xa << 8)
#define REG_SCAN_LIMIT   (0xb << 8)
#define REG_SHUTDOWN     (0xc << 8)
#define REG_DISPLAY_TEST (0xf << 8)

#define VAL_CLEAR 0x00

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static inline uint16_t shuffle(uint16_t val) {
    return (val >> 8) | (val << 8);
}

static esp_err_t sendAll(max7219_t *dev, uint16_t value) {
    uint16_t buf[MAX7219_MAX_CASCADE_SIZE] = { 0 };

    for (uint8_t i = 0; i < dev->cascade_size; i++) {
        buf[i] = shuffle(value);
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = dev->cascade_size * 16;
    t.tx_buffer = buf;
    return spi_device_transmit(dev->spi_dev, &t);
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t max7219_init_desc(max7219_t *dev, spi_host_device_t host, gpio_num_t cs_pin) {
    CHECK_ARG(dev);

    memset(&dev->spi_cfg, 0, sizeof(dev->spi_cfg));
    dev->spi_cfg.spics_io_num = cs_pin;
    dev->spi_cfg.clock_speed_hz = CLOCK_SPEED_HZ;
    dev->spi_cfg.mode = 0;
    dev->spi_cfg.queue_size = 1;
    dev->spi_cfg.flags = SPI_DEVICE_NO_DUMMY;

    return spi_bus_add_device(host, &dev->spi_cfg, &dev->spi_dev);
}

esp_err_t max7219_free_desc(max7219_t *dev) {
    CHECK_ARG(dev);

    return spi_bus_remove_device(dev->spi_dev);
}

esp_err_t max7219_init(max7219_t *dev) {
    CHECK_ARG(dev);
    if (!dev->cascade_size || dev->cascade_size > MAX7219_MAX_CASCADE_SIZE)
    {
        ESP_LOGE(TAG, "Invalid cascade size %d", dev->cascade_size);
        return ESP_ERR_INVALID_ARG;
    }

    // Shutdown all chips
    CHECK(max7219_set_shutdown_mode(dev, true));
    // Disable test
    CHECK(sendAll(dev, REG_DISPLAY_TEST));
    // Set max scan limit
    CHECK(sendAll(dev, REG_SCAN_LIMIT | (ALL_DIGITS - 1)));
    // Clear display
    CHECK(max7219_clear(dev));
    // Set minimal brightness
    CHECK(max7219_set_brightness(dev, 0));
    // Wake up
    CHECK(max7219_set_shutdown_mode(dev, false));

    return ESP_OK;
}

esp_err_t max7219_set_brightness(max7219_t *dev, uint8_t value) {
    CHECK_ARG(dev);
    CHECK_ARG(value <= MAX7219_MAX_BRIGHTNESS);

    CHECK(sendAll(dev, REG_INTENSITY | value));

    return ESP_OK;
}

esp_err_t max7219_set_shutdown_mode(max7219_t *dev, bool shutdown) {
    CHECK_ARG(dev);

    CHECK(sendAll(dev, REG_SHUTDOWN | !shutdown));

    return ESP_OK;
}

esp_err_t max7219_clear(max7219_t *dev) { 
    CHECK_ARG(dev);

    uint8_t val = VAL_CLEAR;
    for (uint8_t i = 0; i < ALL_DIGITS; i++)
        CHECK(sendAll(dev, (REG_DIGIT_0 + ((uint16_t)i << 8)) | val));

    return ESP_OK;
}

esp_err_t max7219_draw_images_8x8(max7219_t *dev, uint8_t count, const void *image) {
    CHECK_ARG(dev && image);

    uint16_t buf[MAX7219_MAX_CASCADE_SIZE] = { 0 };

    for (uint8_t line = 0; line < ALL_DIGITS; line++) {

        uint8_t actualLine = dev->mirrored ? ALL_DIGITS - (line + 1) : line;

        uint16_t cmd = REG_DIGIT_0 + ((uint16_t)actualLine << 8);
        for(uint8_t chip = 0 ; chip < count ; chip++) {
            uint8_t val = *((uint8_t *)image + line + chip * 8);
            buf[dev->mirrored ? count - (chip + 1) : chip] = shuffle(cmd | val);
        }

        spi_transaction_t t = {0};
        t.length = dev->cascade_size * 16;
        t.tx_buffer = buf;
        CHECK(spi_device_transmit(dev->spi_dev, &t));
    }

    return ESP_OK;
}
