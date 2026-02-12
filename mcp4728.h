#ifndef MCP4728_H
#define MCP4728_H

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "rom/ets_sys.h"

// --------------------------------------------
// I2C ADDRESS
// --------------------------------------------

#define MCP_I2C_BASE_ADDRESS 	(0x60)
#define MCP_WRITE_I2C_ADDRESS	(0x60)

// --------------------------------------------
// GENERAL CALL
// --------------------------------------------

#define MCP_GENERAL_CALL_ADDRESS	  (0x00)
#define MCP_GENERAL_CALL_READ_COMMAND (0x0C)
#define MCP_GENERAL_CALL_RESTART	  (0xC1)

// --------------------------------------------
// COMMANDS
// --------------------------------------------

#define MCP_FAST_WRITE		(0x00)
#define MCP_MULTI_WRITE		(0x40)

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

/**
 * @brief Change the I2C address bits of the MCP4728 and store them in EEPROM.
 *
 * 		  This function executes the "Write I2C Address Bits" command (C2=0, C1=1, C0=1). 
 * 		  It uses software bit-banging to precisely pull the LDAC pin LOW during the 
 * 		  8th clock of the second byte. The new address is permanently stored in the 
 * 		  device's EEPROM after the I2C Stop condition.
 *
 * @param scl          GPIO number for the I2C SCL line.
 * @param sda          GPIO number for the I2C SDA line.
 * @param ldac         GPIO number for the MCP4728 LDAC pin.
 * @param current_addr The current 3-bit address (0-7) of the device.
 * @param new_addr     The new 3-bit address (0-7) to be programmed.
 */
void mcp_change_i2c_address(gpio_num_t scl, gpio_num_t sda, gpio_num_t ldac, uint8_t current_addr, uint8_t new_addr);
#endif