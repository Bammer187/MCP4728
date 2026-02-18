/**
 * Changes the I2C programmable addres bits to 100b / 0x04.
 * The current address is read and then changed.
 * 
 *                      PINOUT CONFIGURATION
 * | MCP4728 Pin | Function          | ESP32 Connection | Notes              |
 * |-------------|-------------------|------------------|--------------------|
 * | 1  (VDD)    | Power Supply      | 3.3V             | 2.7V - 5.5V Range  |
 * | 2  (SCL)    | I2C Clock         | GPIO 22          | Req. Pull-up       |
 * | 3  (SDA)    | I2C Data          | GPIO 21          | Req. Pull-up       |
 * | 4  (LDAC)   | Device Selection  | GPIO 27          | -                  |
 * | 10 (VSS)    | Ground            | GND              | Common Ground      |
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mcp4728.h"

static const char *TAG = "MCP_TEST";

#define SCL_GPIO  GPIO_NUM_22
#define SDA_GPIO  GPIO_NUM_21
#define LDAC_PIN  GPIO_NUM_27

void app_main(void)
{
    uint8_t current_address_bits = mcp_read_address_bits(SCL_GPIO, SDA_GPIO, LDAC_PIN);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Current address bits: %d", current_address_bits);

    vTaskDelay(pdMS_TO_TICKS(1000));

    mcp_change_i2c_address(SCL_GPIO, SDA_GPIO, LDAC_PIN, current_address_bits, 4);
    ESP_LOGI(TAG, "Changed address.");

    vTaskDelay(pdMS_TO_TICKS(1000));

    uint8_t new_address_bits = mcp_read_address_bits(SCL_GPIO, SDA_GPIO, LDAC_PIN);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "New address bits: %d", new_address_bits);
}