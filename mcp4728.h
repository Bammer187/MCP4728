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
#define MCP_GENERAL_CALL_UPDATE			(0x08)

// --------------------------------------------
// COMMANDS
// --------------------------------------------

#define MCP_FAST_WRITE					(0x00)
#define MCP_MULTI_WRITE					(0x40)
#define MCP_VREF_WRITE					(0x80)
#define MCP_PD_WRITE					(0xA0)
#define MCP_GAIN_WRITE					(0xC0)

// --------------------------------------------
// COMMANDS
// --------------------------------------------

#define MCP_VREF_VDD					(0x00)
#define MCP_VREF_INTERNAL				(0x01)

typedef enum {
	MCP_CHANNEL_A,
	MCP_CHANNEL_B,
	MCP_CHANNEL_C,
	MCP_CHANNEL_D
}mcp4728_channel_t;

typedef enum {
	MCP_PD_NORMAL,
	MCP_PD_1_K,
	MCP_PD_100_K,
	MCP_PD_500_K
}mcp4728_pd_t;

typedef enum {
    MCP_GAIN_ONE,
    MCP_GAIN_TWO
}mcp4728_gain_t;

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
 * 		  - ESP_ERR_INVALID_ARG if mcp or mcp dev-handle is NULL.
 * 		  - ESP_FAIL on I2C communication error.
 */
esp_err_t mcp_fast_write_channels(mcp4728_t *mcp, mcp4728_pd_t pd, uint16_t chA, uint16_t chB, uint16_t chC, uint16_t chD);

/**
 * @brief Write configuration and voltage data to a single DAC channel.
 *
 * 		  This function uses the Multi-Write command to update a specific channel's 
 * 		  voltage along with its configuration bits, including voltage reference, 
 * 		  power-down mode, and gain. The output is updated immediately after 
 * 		  the transmission.
 *
 * @param mcp      Pointer to the MCP4728 device structure.
 * @param channel  The target channel to update (0=A, 1=B, 2=C, 3=D).
 * @param vref     Voltage reference selection (false = VDD, true = Internal).
 * @param pd       Power-down mode bits (0=Normal, 1=1kOhm, 2=100kOhm, 3=500kOhm).
 * @param gain     Gain selection (false = 1x, true = 2x).
 * @param data     12-bit raw DAC value (0-4095).
 *
 * @return 
 * 		  - ESP_OK on success.
 * 		  - ESP_ERR_INVALID_ARG if mcp or mcp dev-handle is NULL.
 * 		  - ESP_FAIL on I2C communication error.
 */
esp_err_t mcp_multi_write_channel(mcp4728_t *mcp, mcp4728_channel_t channel, bool vref, mcp4728_pd_t pd, mcp4728_gain_t gain, uint16_t data);

/**
 * @brief Configure the voltage reference source for each DAC channel.
 *
 * 		  This function sets the VREF bit for each of the four channels. Each channel 
 * 		  can independently choose between an external reference (VDD) or the 
 *		  internal 2.048V precision voltage reference.
 *
 * @param mcp    Pointer to the MCP4728 device structure.
 * @param vrefA  Reference for Ch A (false = VDD, true = Internal 2.048V).
 * @param vrefB  Reference for Ch B.
 * @param vrefC  Reference for Ch C.
 * @param vrefD  Reference for Ch D.
 *
 * @return 
 * 		  - ESP_OK on success.
 * 		  - ESP_ERR_INVALID_ARG if mcp or mcp dev-handle is NULL.
 * 		  - ESP_FAIL on I2C communication error.
 */
esp_err_t mcp_set_vref(mcp4728_t *mcp, bool vrefA, bool vrefB, bool vrefC, bool vrefD);

/**
 * @brief Configure the power-down modes for all four DAC channels.
 *
 * 		  This function updates the power-down (PD) bits for each channel, allowing 
 * 		  them to be set to Normal mode or one of three power-down levels with 
 * 		  different internal resistive loads to ground (1kΩ, 100kΩ, or 500kΩ). 
 * 		  This is useful for reducing power consumption when specific outputs are not in use.
 *
 * @param mcp    Pointer to the MCP4728 device structure.
 * @param pdA    Power-down mode for Channel A (using mcp4728_pd_t enum).
 * @param pdB    Power-down mode for Channel B.
 * @param pdC    Power-down mode for Channel C.
 * @param pdD    Power-down mode for Channel D.
 *
 * @return 
 * 		  - ESP_OK on success.
 * 		  - ESP_ERR_INVALID_ARG if mcp or mcp dev-handle is NULL.
 * 		  - ESP_FAIL on I2C communication error.
 */
esp_err_t mcp_set_power_down(mcp4728_t *mcp, mcp4728_pd_t pdA, mcp4728_pd_t pdB, mcp4728_pd_t pdC, mcp4728_pd_t pdD);

/**
 * @brief Set the gain selection bits for all four DAC channels.
 *
 * 		  This function updates the gain configuration (1x or 2x) for all channels.
 * 		  This is a fast single-byte command that does not affect the voltage 
 *		  or VREF settings.
 *
 * @param mcp    Pointer to the MCP4728 device structure.
 * @param gainA  Gain selection for Channel A (MCP_GAIN_ONE or MCP_GAIN_TWO).
 * @param gainB  Gain selection for Channel B.
 * @param gainC  Gain selection for Channel C.
 * @param gainD  Gain selection for Channel D.
 *
 * @return 
 * 		  - ESP_OK on success.
 *		  - ESP_ERR_INVALID_ARG if mcp or mcp dev-handle is NULL.
 * 		  - ESP_FAIL on I2C communication error.
 */
esp_err_t mcp_set_gains(mcp4728_t *mcp, mcp4728_gain_t gainA, mcp4728_gain_t gainB, mcp4728_gain_t gainC, mcp4728_gain_t gainD);

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