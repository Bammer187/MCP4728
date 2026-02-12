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

/**
 * @brief Initialize the MCP4728 device structure and add it to the I2C bus.
 *
 * 		  This function initializes the MCP4728 driver structure and registers 
 * 		  the device onto the I2C master bus. It configures the device address 
 * 		  (including the programmable address bits) and sets the bus frequency 
 * 		  for communication with the 4-channel 12-bit DAC.
 *
 * @param mcp          Pointer to the MCP4728 device structure.
 * @param bus_handle   Pointer to the I2C master bus handle.
 * @param addr_bits    The 3-bit programmable address portion (A2, A1, A0).
 * @param i2c_freq     I2C clock speed in Hz.
 */
void mcp4728_init(mcp4728_t *mcp, i2c_master_bus_handle_t *bus_handle, uint8_t addr_bits, uint32_t i2c_freq);

/**
 * @brief Read the programmable I2C address bits from the MCP4728 using the LDAC pin.
 *
 * 		  This function performs the specific hardware handshake required to read 
 * 		  the EEPROM-stored address bits of the MCP4728.
 * 
 * 		  Note: This is typically used when the physical address bits are unknown.
 *
 * @param scl          GPIO number used for the I2C SCL line.
 * @param sda          GPIO number used for the I2C SDA line.
 * @param ldac         GPIO number connected to the MCP4728 LDAC pin.
 *
 * @return 
 * 		  - The 3-bit address (0x00 to 0x07)
 */
uint8_t mcp_read_address_bits(gpio_num_t scl, gpio_num_t sda, gpio_num_t ldac);

#endif