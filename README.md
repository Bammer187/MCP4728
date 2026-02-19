# MCP4728 Library for ESP-IDF (v5.x)
A lightweight, ESP-IDF compatible C driver for the MCP4728, a 12-bit, quad-channel Digital-to-Analog Converter (DAC) with non-volatile memory (EEPROM).

## Table of Contents
  - [Features](#features)
  - [Installation](#installation)
  - [License](#license)

## Features

- Modern ESP-IDF: Uses the new i2c_master driver (v5.0+).
- Quad Channel Support: Independent control for Channels A, B, C, and D.
- Flexible VREF: Toggle between internal (2.048V) and external (VDD) voltage references.
- Multiple Write Modes: Support for Single Write, Fast Write, Multi Write and Sequential Write.

## Installation

### Manual Integration

Copy the source files (`mcp4728.c`, `mcp4728.h`, `CMakeLists.txt`) into your project's components or main directory. Ensure the header file is accessible within your include path.

### Add as submodule
`git submodule add https://github.com/Bammer187/MCP4728.git components/mcp4728`

Don't forget to update your `CMakeLists.txt`!

## License

[MIT License](LICENSE)
