#ifndef MCP4728_H
#define MCP4728_H

#include "driver/gpio.h"
#include "rom/ets_sys.h"

#define MCP_I2C_BASE_ADDRESS (0x60)

#define MCP_GENERAL_CALL_ADDRESS	  (0x00)
#define MCP_GENERAL_CALL_READ_COMMAND (0x0C)
#define MCP_GENERAL_CALL_RESTART	  (0xC1)

#endif