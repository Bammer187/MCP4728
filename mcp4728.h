#ifndef MCP4728_H
#define MCP4728_H

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "rom/ets_sys.h"

#define MCP_I2C_BASE_ADDRESS (0x60)

#define MCP_GENERAL_CALL_ADDRESS	  (0x00)
#define MCP_GENERAL_CALL_READ_COMMAND (0x0C)
#define MCP_GENERAL_CALL_RESTART	  (0xC1)

typedef struct {
	i2c_master_dev_handle_t dev_handle;
}mcp4728_t;

void mcp4728_init(mcp4728_t *mcp, i2c_master_bus_handle_t *bus_handle, uint8_t addr_bits, uint32_t i2c_freq);
uint8_t mcp_read_address_bits(gpio_num_t scl, gpio_num_t sda, gpio_num_t ldac);

#endif