#ifndef MCP4728_H
#define MCP4728_H

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "rom/ets_sys.h"

#define MCP_I2C_TIMEOUT_MS 5000

// --------------------------------------------
// I2C ADDRESS
// --------------------------------------------

#define MCP_I2C_BASE_ADDRESS 			(0x60)
#define MCP_WRITE_I2C_ADDRESS			(0x60)

// --------------------------------------------
// GENERAL CALL
// --------------------------------------------

#define MCP_GENERAL_CALL_ADDRESS	  	(0x00)
#define MCP_GENERAL_CALL_READ_COMMAND 	(0x0C)
#define MCP_GENERAL_CALL_RESTART	 	(0xC1)

// --------------------------------------------
// COMMANDS
// --------------------------------------------

#define MCP_FAST_WRITE					(0x00)
#define MCP_MULTI_WRITE					(0x40)

// --------------------------------------------
// POWER-DOWN MODES
// --------------------------------------------

#define MCP_PD_NORMAL					(0x00)
#define MCP_PD_1_K						(0x01)
#define MCP_PD_100_K					(0x02)
#define MCP_PD_500_K					(0x03)

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
 * @brief Update all four DAC output channels using the Fast Write command.
 *
 * 		  This function quickly updates the DAC registers for channels A, B, C, and D 
 * 		  in a single I2C transaction. It uses the "Fast Write" protocol, which 
 * 		  minimizes bus overhead but does not modify the EEPROM or configuration 
 * 		  bits (such as Voltage Reference or Gain).
 * 
 * 		  Note: The input values are masked to 12-bit (0-4095).
 *
 * @param mcp    Pointer to the MCP4728 device structure.
 * @param chA    12-bit value for Channel A.
 * @param chB    12-bit value for Channel B.
 * @param chC    12-bit value for Channel C.
 * @param chD    12-bit value for Channel D.
 *
 * @return 
 * 		  - ESP_OK on success.
 * 		  - ESP_FAIL on I2C communication error.
 */
esp_err_t mcp_fast_write_channels(mcp4728_t *mcp, uint16_t chA, uint16_t chB, uint16_t chC, uint16_t chD);

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