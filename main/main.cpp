#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_https_ota.h"
#include "nvs_flash.h"
#include "lwip/apps/sntp.h"
#include "wifi_helper.h"
#include "mqtt_helper.h"
#include "smbus.h"
#include "i2c-lcd1602.h"
#include "u8g2_esp32_hal.h"
#include "u8g2.h"

static const char *TAG = "app";

// LCD1602
#define LCD_NUM_ROWS               2
#define LCD_NUM_COLUMNS            32
#define LCD_NUM_VISIBLE_COLUMNS    16
#define LCD1602_I2C_ADDRESS        0x27

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        GPIO_NUM_21
#define I2C_MASTER_SCL_IO        GPIO_NUM_22

static i2c_lcd1602_info_t * lcd_info;

// MAX7219
#define SCROLL_DELAY 50
#define CASCADE_SIZE 4

#define HOST VSPI_HOST

#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

static u8g2_t u8g2; // a structure which will contain all the data for one display

static const char* ota_url = "http://raspberrypi.fritz.box:8032/esp32/1602.bin";

static volatile uint8_t brightness = 0;

static void ota_task(void * pvParameter) {
    ESP_LOGI(TAG, "Starting OTA update...");

    esp_http_client_config_t config = {
        .url = ota_url,
    };
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware Upgrades Failed");
    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void max_7219_task(void *pvParameter) {

	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.host = HOST;
    u8g2_esp32_hal.clock_speed_hz = SPI_MASTER_FREQ_10M;
	u8g2_esp32_hal.clk   = PIN_NUM_CLK;
	u8g2_esp32_hal.mosi  = PIN_NUM_MOSI;
	u8g2_esp32_hal.cs    = PIN_NUM_CS;
	u8g2_esp32_hal_init(u8g2_esp32_hal);

	u8g2_Setup_max7219_32x8_f(
		&u8g2,
		U8G2_R2,
		u8g2_esp32_spi_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure

	u8g2_InitDisplay(&u8g2);
	u8g2_SetContrast(&u8g2, 0); // Sets the brightness, start low
    u8g2_ClearDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0); // Init display leaves the display off

    // https://raw.githubusercontent.com/wiki/olikraus/u8g2/fntpic/u8g2_font_victoriabold8_8r.png
    // u8g2_SetFont(&u8g2, u8g2_font_victoriabold8_8r);
    // https://raw.githubusercontent.com/wiki/olikraus/u8g2/fntpic/u8g2_font_pxplusibmcga_8f.png
    // u8g2_SetFont(&u8g2, u8g2_font_pxplusibmcga_8f);
    // https://raw.githubusercontent.com/wiki/olikraus/u8g2/fntpic/u8g2_font_nokiafc22_tf.png
    u8g2_SetFont(&u8g2, u8g2_font_nokiafc22_tf);


    const char *str = "    Hello from U8g2... Arduino monochrome graphics library    ";

    const size_t charHeight = u8g2_GetMaxCharHeight(&u8g2);
    const size_t bufLen = 12;

    char buf[bufLen];

    size_t charPos = 0;
    size_t innerPos = 0;
    size_t charWidth = u8g2_GetGlyphWidth(&u8g2, str[charPos]);

    while (1) {
        u8g2_SetContrast(&u8g2, brightness << 4);
        u8g2_ClearBuffer(&u8g2);					// clear the internal memory


        if(innerPos == charWidth) {
            charPos++;
            if(charPos == strlen(str)) {
                charPos = 0;
            }

            innerPos = 0;
            charWidth = u8g2_GetGlyphWidth(&u8g2, str[charPos]);
        }
        for(uint16_t i = 0; i < bufLen-1; i++) {
            buf[i] = str[charPos+i];
            if ( str[charPos+i] == '\0' ) {
                break;
            }
        }
        buf[bufLen-1] = '\0';
        u8g2_DrawStr(&u8g2, -innerPos, charHeight - 1, buf);
        innerPos++;


        u8g2_SendBuffer(&u8g2);					// transfer internal memory to the display
        vTaskDelay(pdMS_TO_TICKS(SCROLL_DELAY));
    }

    vTaskDelete(NULL);
}


static esp_err_t i2c_master_init() {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups (And we are using level converter with pullup)
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups (And we are using level converter with pullup)
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_LEN, I2C_MASTER_TX_BUF_LEN, 0);
}

static void lcd1602_setup() {
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = LCD1602_I2C_ADDRESS;
 
    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

    // Set up the LCD1602 device with backlight off
    lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                     LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));
    
    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));

    ESP_ERROR_CHECK(i2c_lcd1602_set_backlight(lcd_info, true));
}

static void subscribeTopics() {
    subscribeDevTopic("display/write");
    subscribeDevTopic("leds/brightness");
    subscribeDevTopic("$update");
}

static uint8_t hexDigitValue(uint8_t input) {
    if(input >= '0' && input <= '9') {
        return input - '0';
    }

    if(input >= 'a' && input <= 'f') {
        return 10 + (input- 'a');
    }

    if(input >= 'A' && input <= 'Z') {
        return 10 + (input - 'A');
    }

    return 0;
}

static void handleMessage(const char* topic1, const char* topic2, const char* topic3, const char* data) {

    if(
        strcmp(topic1, "$update") == 0 && 
        topic2 == NULL && 
        topic3 == NULL
    ) {
        xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
    }

    if(
        strcmp(topic1, "leds") == 0 && 
        strcmp(topic2, "brightness") == 0 && 
        topic3 == NULL
    ) {
        brightness = (uint8_t)strtol(data, NULL, 10);
    }

    if(
        strcmp(topic1, "display") == 0 && 
        strcmp(topic2, "write") == 0 && 
        topic3 == NULL
    ) {
        size_t dataPos = 0;

        for(int row = 0 ; row < 2 ; row++) {
            i2c_lcd1602_move_cursor(lcd_info, 0, row);
            uint32_t pos = 0;
            for(; pos < 16; pos++) {
                // Skip spaces
                while(dataPos < strlen(data) && data[dataPos] == ' ') {
                    dataPos++;
                }

                // Next line
                if(dataPos < strlen(data) && data[dataPos] == ',') {
                    // Don't move, since we'll want to re-read the comma when gobbling up leftovers.
                    break;
                }

                if(dataPos == strlen(data)) {
                    break;
                }
                // Read first digit.
                uint8_t valLeft = hexDigitValue(data[dataPos]);
                dataPos++;

                if(dataPos == strlen(data)) {
                    break;
                }
                // Read second digit.
                uint8_t valRight = hexDigitValue(data[dataPos]);
                dataPos++;

                i2c_lcd1602_write_char(lcd_info, valLeft * 0x10 + valRight);
            }
            for(; pos < 16; pos++) {
                i2c_lcd1602_write_char(lcd_info, 0x20);
            }
            // Gobble up any remainder on this line
            while(dataPos < strlen(data)) {
                dataPos++;
                if(data[dataPos-1] == ',') {
                    break;
                }
            }
        }
    }
}

extern "C" void app_main() {

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(i2c_master_init());

    lcd1602_setup();

    xTaskCreatePinnedToCore(max_7219_task, "max_7219_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, APP_CPU_NUM);


    i2c_lcd1602_clear(lcd_info);
    i2c_lcd1602_write_string(lcd_info, "Init WiFi...");

    wifiStart();

    wifiWait();


    i2c_lcd1602_clear(lcd_info);
    i2c_lcd1602_write_string(lcd_info, "Init SNTP...");

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();


    i2c_lcd1602_clear(lcd_info);
    i2c_lcd1602_write_string(lcd_info, "Init MQTT...");

    mqttStart(subscribeTopics, handleMessage);
    mqttWait();

    char text[81] = {};
    i2c_lcd1602_clear(lcd_info);
    tcpip_adapter_ip_info_t ipInfo = {}; 
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);
    snprintf(text, sizeof(text),"%s",ip4addr_ntoa(&ipInfo.ip));
    i2c_lcd1602_write_string(lcd_info, text);

    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    snprintf(text, sizeof(text),"Free: %d", esp_get_minimum_free_heap_size());
    i2c_lcd1602_write_string(lcd_info, text);
}
