# Zalmotek RA2E1 Feather AS7331

UV spectrum sensor integration with Zalmotek's RA2E1 Feather board powered by Renesas microcontroller technology.

## Overview

This project demonstrates how to interface the AS7331 UV spectrum sensor with a Zalmotek RA2E1 Feather board. The implementation uses I2C communication to collect UV light measurements across UVA, UVB, and UVC bands, along with temperature readings, utilizing Zalmotek hardware with Renesas RA2E1 microcontroller technology.

## Hardware Requirements

- Zalmotek RA2E1 Feather board 
- AS7331 UV spectrum sensor
- I2C connection cables
- Power supply or USB connection for the Feather board

## Software Requirements

- Renesas e² studio IDE
- Renesas FSP (Flexible Software Package)
- GCC ARM compiler
- J-Link debugger

## Features

- UVA, UVB, and UVC band light intensity measurement
- Temperature sensing
- Configurable measurement modes (continuous, command, synchronized)
- Adjustable gain and timing parameters
- I2C communication interface

## Code Functionality

The main application:
- Initializes the I2C interface for communication with the AS7331 sensor
- Configures the sensor with appropriate mode, clock, gain, and timing parameters
- Continuously reads UV and temperature measurements from the sensor
- Converts raw sensor data to physical units (µW/cm²)
- Displays the measurements through the debug console

## Getting Started

### Setup

1. Connect the AS7331 sensor to the Zalmotek RA2E1 Feather board via I2C
2. Open the project in Renesas e² studio
3. Build the project using the GCC ARM compiler
4. Flash the compiled firmware to the RA2E1 Feather board using J-Link
5. Connect to the debug console to view sensor readings

### Configuration

The sensor can be configured by modifying these parameters in hal_entry.cpp:

```c
// Specify sensor parameters
MMODE   mmode = AS7331_CONT_MODE;  // Measurement mode (CONT, CMD, SYNS, SYND)
CCLK    cclk  = AS7331_1024;       // Internal clock (1.024, 2.048, 4.096, 8.192 MHz)
uint8_t sb    = 0x01;              // Standby mode (0x01 enabled, 0x00 disabled)
uint8_t gain  = 8;                 // ADC gain (0-11, higher values for lower light)
uint8_t timeMs = 9;                // Integration time (2^time in ms)
```

## Project Structure

- `src/hal_entry.cpp`: Main application entry point and sensor reading logic
- `src/as7331.h`: AS7331 sensor driver implementation
- `src/i2c_dev.h`: I2C device communication interface
- `src/Wire.h`: Arduino-like I2C wrapper implementation
- `src/SEGGER_RTT/`: Debug console output implementation

## License

Copyright © 2023 Zalmotek

## Additional Resources

- [Zalmotek Website](https://zalmotek.com)
- [Zalmotek RA2E1 Website](https://zalmotek.com/products/RA2E1-Feather-SoM/)
- [SEGGER RTT Documentation](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) 