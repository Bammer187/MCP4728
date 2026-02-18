/**
 * Continuously increments a 12-bit counter and updates Channel A using 
 * the Single Write command. The output voltage is calculated based on a 
 * 3.3V VDD reference and logged to the console alongside the raw DAC value.
 * 
 *                      PINOUT CONFIGURATION
 * | MCP4728 Pin | Function      | ESP32 Connection | Notes              |
 * |-------------|---------------|------------------|--------------------|
 * | 1  (VDD)    | Power Supply  | 3.3V             | 2.7V - 5.5V Range  |
 * | 2  (SCL)    | I2C Clock     | GPIO 22          | Req. Pull-up       |
 * | 3  (SDA)    | I2C Data      | GPIO 21          | Req. Pull-up       |
 * | 6  (VOUTA)  | DAC Output A  | -                | Output Voltage     |
 * | 10 (VSS)    | Ground        | GND              | Common Ground      |
 */

#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "mcp4728.h"

static const char *TAG = "MCP_TEST";

#define SCL_GPIO  GPIO_NUM_22
#define SDA_GPIO  GPIO_NUM_21

#define I2C_FREQUENZY 100000
#define I2C_ADDRESS 0 // <=7

float voltage_steps = 3.3f / 4096.0f;

void app_main(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    mcp4728_t mcp;
    mcp4728_init(&mcp, &bus_handle, I2C_ADDRESS, I2C_FREQUENZY);

    uint16_t counter = 0;

    mcp4728_channel_config_t mcp_config = {
        .channel = MCP_CHANNEL_A,
        .vref = 0,
        .pd = MCP_PD_NORMAL,
        .gain = MCP_GAIN_ONE,
        .data = 0,
    };

    while(1) {
        if(counter++ == 4000) counter = 0;
        mcp_config.data = counter;
        mcp_single_write(&mcp, &mcp_config);
        ESP_LOGI(TAG, "DAC VALUE: %d | VOLTAGE: %f", counter, counter * voltage_steps);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}