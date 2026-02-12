#include "mcp4728.h"

static void sw_i2c_write_byte(gpio_num_t scl, gpio_num_t sda, gpio_num_t ldac, uint8_t byte, uint32_t delay, bool trigger_ldac) {
    for (int i = 0; i < 8; i++) {
        gpio_set_level(sda, (byte << i) & 0x80 ? 1 : 0);
        
        // Pull LDAC low durning eigth bit
        if (i == 7 && trigger_ldac) {
            gpio_set_level(ldac, 0);
        }
        
        ets_delay_us(delay);
        gpio_set_level(scl, 1);
        ets_delay_us(delay);
        gpio_set_level(scl, 0);
    }

    // ACK Phase
    gpio_set_level(sda, 1);
    gpio_set_direction(sda, GPIO_MODE_INPUT);
    ets_delay_us(delay);
    gpio_set_level(scl, 1);
    ets_delay_us(delay);
    gpio_set_level(scl, 0);
    gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT);
}


static uint8_t sw_i2c_read_byte(gpio_num_t scl, gpio_num_t sda, uint32_t delay, bool ack) {
    uint8_t byte = 0;
    gpio_set_direction(sda, GPIO_MODE_INPUT);
    
    for (int i = 0; i < 8; i++) {
        ets_delay_us(delay);
        gpio_set_level(scl, 1);
        if (gpio_get_level(sda)) byte |= (0x80 >> i);
        ets_delay_us(delay);
        gpio_set_level(scl, 0);
    }

    // ACK Phase
    gpio_set_direction(sda, GPIO_MODE_OUTPUT);
    gpio_set_level(sda, ack ? 0 : 1);
    ets_delay_us(delay);
    gpio_set_level(scl, 1);
    ets_delay_us(delay);
    gpio_set_level(scl, 0);
    return byte;
}


uint8_t mcp_read_address_bits(gpio_num_t scl, gpio_num_t sda, gpio_num_t ldac)
{
    uint32_t frequenzy = 100000;
    uint32_t delay = 1000000 / (frequenzy * 2);

    gpio_set_direction(scl, GPIO_MODE_OUTPUT);
    gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(ldac, GPIO_MODE_OUTPUT);

    gpio_set_level(ldac, 1);
    gpio_set_level(scl, 0);
    gpio_set_level(sda, 0);
    ets_delay_us(delay);

    sw_i2c_write_byte(scl, sda, ldac, MCP_GENERAL_CALL_ADDRESS, delay, false);
    sw_i2c_write_byte(scl, sda, ldac, MCP_GENERAL_CALL_READ_COMMAND, delay, true);

    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    ets_delay_us(delay);
    gpio_set_level(sda, 0);
    ets_delay_us(delay);
    gpio_set_level(scl, 0);

    sw_i2c_write_byte(scl, sda, ldac, MCP_GENERAL_CALL_RESTART, delay, false);

    uint8_t data = sw_i2c_read_byte(scl, sda, delay, false);

    gpio_set_level(sda, 0);
    ets_delay_us(delay);
    gpio_set_level(scl, 1);
    ets_delay_us(delay);
    gpio_set_level(ldac, 1);

    return (data >> 5) & 0x07;
}