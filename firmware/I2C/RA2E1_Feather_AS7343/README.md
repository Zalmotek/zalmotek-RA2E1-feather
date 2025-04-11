# Zalmotek RA2E1 Feather AS7343

Spectral sensor integration with Zalmotek RA2E1 Feather board powered by Renesas RA microcontroller.

## Overview

This project demonstrates how to interface the AS7343 14-channel spectral sensor with the Zalmotek RA2E1 Feather board. The implementation uses I2C communication to configure the sensor and read spectral data across multiple wavelengths from 405nm to 855nm (visible light to NIR).

## Hardware Requirements

- Zalmotek RA2E1 Feather board 
- AMS OSRAM AS7343 14-Channel Spectral Sensor
- I2C connection cables
- USB cable for programming and power

## Software Requirements

- e² studio IDE
- Renesas FSP (Flexible Software Package)
- C/C++ compiler
- J-Link debugger software

## Features

- Configure and initialize the AS7343 spectral sensor
- Set integration time, gain, and LED control
- Read all spectral channels (wavelengths from 405nm to 855nm)
- Print measurement results via serial debug output
- Customizable sensor configuration (gain, integration time)

## Code Functionality

The main application:
- Initializes the AS7343 14-channel spectral sensor via I2C
- Configures sensor parameters (ATIME=100, ASTEP=999, GAIN=256X)
- Continuously reads all spectral channels data in a loop
- Processes and outputs readings for each wavelength (415nm to 680nm)
- Displays Clear and NIR channel data
- Reports any errors during sensor communication

## Getting Started

### Setup

1. Connect the AS7343 sensor to the RA2E1 Feather board via I2C pins
2. Install e² studio IDE and the Renesas FSP
3. Import the project into e² studio
4. Build the project
5. Connect the board via USB and flash the firmware using J-Link

### Configuration

You can modify sensor parameters in the hal_entry.cpp file:

```c
// Example configuration
as7343.setATIME(100);    // Sets ADC integration time
as7343.setASTEP(999);    // Sets integration steps
as7343.setGain(AS7343_GAIN_256X);  // Sets sensor gain
```

## Project Structure

- `src/hal_entry.cpp`: Main application code with sensor initialization and reading logic
- `src/AMS_OSRAM_AS7343.h`: Header file for the AS7343 sensor library
- `src/i2c_op.h`: I2C operation functions for communicating with the sensor
- `src/Wire.h`: I2C communication wrapper library

## License

Copyright © 2023 Zalmotek

## Additional Resources

- [Zalmotek Website](https://zalmotek.com)
- [Zalmotek RA2E1 Website](https://zalmotek.com/products/RA2E1-Feather-SoM/)
- [SEGGER RTT Documentation](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) 