#include "mcp4728.h"

void mcp4728_init(mcp4728_t *mcp, i2c_master_bus_handle_t *bus_handle, uint8_t addr_bits, uint32_t i2c_freq)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MCP_I2C_BASE_ADDRESS | (addr_bits & 0x07),
        .scl_speed_hz = i2c_freq,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_cfg, &mcp->dev_handle));
}


esp_err_t mcp_fast_write(mcp4728_t *mcp, mcp4728_pd_t pd, uint16_t chA, uint16_t chB, uint16_t chC, uint16_t chD)
{
    if (mcp == NULL || mcp->dev_handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t b[8];

    uint8_t pd_bits = ((pd & 0x03) << 4);

    // Channel A
    b[0] = MCP_FAST_WRITE | pd_bits | (uint8_t)((chA >> 8) & 0x0F); 
    b[1] = (uint8_t)(chA & 0xFF);

    // Channel B
    b[2] = pd_bits | (uint8_t)((chB >> 8) & 0x0F);
    b[3] = (uint8_t)(chB & 0xFF);

    // Channel C
    b[4] = pd_bits | (uint8_t)((chC >> 8) & 0x0F);
    b[5] = (uint8_t)(chC & 0xFF);

    // Channel D
    b[6] = pd_bits | (uint8_t)((chD >> 8) & 0x0F);
    b[7] = (uint8_t)(chD & 0xFF);

    return i2c_master_transmit(mcp->dev_handle, b, sizeof(b), MCP_I2C_TIMEOUT_MS);
}


esp_err_t mcp_multi_write(mcp4728_t *mcp, uint8_t num_channels, mcp4728_channel_config_t *configs)
{
    if (mcp == NULL || mcp->dev_handle == NULL || num_channels > 4) return ESP_ERR_INVALID_ARG;

    uint8_t b[12];
    
    for (int i = 0; i < num_channels; i++) {
        uint8_t offset = i * 3;
        b[offset] = MCP_MULTI_WRITE | ((configs[i].channel & 0x03) << 1);

        b[offset + 1] = (configs[i].vref << 7) | 
                        ((configs[i].pd & 0x03) << 5) | 
                        (configs[i].gain << 4) | 
                        (uint8_t)((configs[i].data >> 8) & 0x0F);

        b[offset + 2] = (uint8_t)(configs[i].data & 0xFF);
    }

    return i2c_master_transmit(mcp->dev_handle, b, num_channels * 3, MCP_I2C_TIMEOUT_MS);
}


esp_err_t mcp_single_write(mcp4728_t *mcp, mcp4728_channel_config_t *config)
{
    if (mcp == NULL || mcp->dev_handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t b[3];

    b[0] =  MCP_SINGLE_WRITE | ((config->channel & 0x03) << 1);

    b[1] =  (config->vref << 7) | 
            ((config->pd & 0x03) << 5) | 
            (config->gain << 4) | 
            (uint8_t)((config->data >> 8) & 0x0F);

    b[2] =  (uint8_t)(config->data & 0xFF);

    return i2c_master_transmit(mcp->dev_handle, b, sizeof(b), MCP_I2C_TIMEOUT_MS);
}


esp_err_t mcp_seq_write(mcp4728_t *mcp, mcp4728_channel_config_t *config)
{
    if (mcp == NULL || mcp->dev_handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t b[10];

    b[0] = MCP_SEQ_WRITE | ((config->channel & 0x03) << 1);

    for(int i = 0; i < 4 - config->channel; i++) {
        uint8_t offset = i * 2;
        b[offset + 1]   =   (config->vref << 7) | 
                            ((config->pd & 0x03) << 5) | 
                            (config->gain << 4) | 
                            (uint8_t)((config->data >> 8) & 0x0F);

        b[offset + 2] =     (uint8_t)(config->data & 0xFF);
    }

    return i2c_master_transmit(mcp->dev_handle, b, 3 + config->channel * 2, MCP_I2C_TIMEOUT_MS);
}


esp_err_t mcp_set_vref(mcp4728_t *mcp, bool vrefA, bool vrefB, bool vrefC, bool vrefD)
{
    if (mcp == NULL || mcp->dev_handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t b[1];

    b[0] = MCP_GAIN_WRITE | ((gainA & 0x01) << 3) | ((gainB & 0x01) << 2) | ((gainC & 0x01) << 1) | (gainD & 0x01);
    
    return i2c_master_transmit(mcp->dev_handle, b, 1, MCP_I2C_TIMEOUT_MS);
}


esp_err_t mcp_set_power_down(mcp4728_t *mcp, mcp4728_pd_t pdA, mcp4728_pd_t pdB, mcp4728_pd_t pdC, mcp4728_pd_t pdD)
{
    if (mcp == NULL || mcp->dev_handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t b[2];

    b[0] = MCP_PD_WRITE | ((pdA & 0x03) << 2) | ((pdB & 0x03) << 2);
    b[1] = ((pdC & 0x03) << 6) | ((pdD & 0x03) << 4);

    return i2c_master_transmit(mcp->dev_handle, b, sizeof(b), MCP_I2C_TIMEOUT_MS);
}


esp_err_t mcp_set_gains(mcp4728_t *mcp, mcp4728_gain_t gainA, mcp4728_gain_t gainB, mcp4728_gain_t gainC, mcp4728_gain_t gainD)
{
    if (mcp == NULL || mcp->dev_handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t b = MCP_GAIN_WRITE | ((gainA & 0x01) << 3) | ((gainB & 0x01) << 2) | ((gainC & 0x01) << 1) | (gainD & 0x01);
    
    return i2c_master_transmit(mcp->dev_handle, b, 1, MCP_I2C_TIMEOUT_MS);
}


static void sw_i2c_write_byte(gpio_num_t scl, gpio_num_t sda, gpio_num_t ldac, uint8_t byte, uint32_t delay, bool trigger_ldac)
{
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


static uint8_t sw_i2c_read_byte(gpio_num_t scl, gpio_num_t sda, uint32_t delay, bool ack)
{
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


void mcp_change_i2c_address(gpio_num_t scl, gpio_num_t sda, gpio_num_t ldac, uint8_t current_addr, uint8_t new_addr)
{
    uint32_t frequenzy = 100000;
    uint32_t delay = 1000000 / (frequenzy * 2);

    gpio_set_direction(scl, GPIO_MODE_OUTPUT);
    gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(ldac, GPIO_MODE_OUTPUT);

    gpio_set_level(ldac, 1);
    gpio_set_level(sda, 0);
    gpio_set_level(scl, 0);
    ets_delay_us(delay);

    uint8_t byte1 = (MCP_I2C_BASE_ADDRESS | (current_addr & 0x07)) << 1;

    sw_i2c_write_byte(scl, sda, ldac, byte1, delay, false);

    uint8_t byte2 = MCP_WRITE_I2C_ADDRESS | ((current_addr & 0x07) << 2) | 0x01;

    sw_i2c_write_byte(scl, sda, ldac, byte2, delay, true);

    uint8_t byte3 = MCP_WRITE_I2C_ADDRESS | ((new_addr & 0x07) << 2) | 0x02;

    sw_i2c_write_byte(scl, sda, ldac, byte3, delay, false);

    uint8_t byte4 = MCP_WRITE_I2C_ADDRESS | ((new_addr & 0x07) << 2) | 0x03;

    sw_i2c_write_byte(scl, sda, ldac, byte4, delay, false);

    gpio_set_level(sda, 0);
    ets_delay_us(delay);
    gpio_set_level(scl, 1);
    ets_delay_us(delay);
    gpio_set_level(ldac, 1);
}
