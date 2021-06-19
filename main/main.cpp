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
#include "nvs_flash.h"
#include "lwip/apps/sntp.h"
#include "wifi_helper.h"
#include "mqtt_helper.h"
#include "smbus.h"
#include "i2c-lcd1602.h"

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
#define I2C_MASTER_SDA_IO        18
#define I2C_MASTER_SCL_IO        19

static i2c_lcd1602_info_t * lcd_info;

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
                    dataPos++;
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
