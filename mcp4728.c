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
