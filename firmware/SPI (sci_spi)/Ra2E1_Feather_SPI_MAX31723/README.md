# Zalmotek Ra2E1 Feather SPI MAX31723

Temperature monitoring solution using the Zalmotek Ra2E1 Feather board powered by Renesas RA2E1 microcontroller with the MAX31723 digital temperature sensor via SPI.

## Overview

This project demonstrates SPI communication between the Zalmotek Ra2E1 Feather board and a MAX31723 digital temperature sensor. The application continuously reads temperature data from the sensor and outputs the values through serial communication at 115200 baud rate.

## Hardware Requirements

- Zalmotek Ra2E1 Feather board 
- MAX31723 digital temperature sensor 
- Connecting wires

## Software Requirements

- Renesas e² studio IDE
- Renesas FSP (Flexible Software Package)
- Terminal emulator (for viewing temperature readings)

## Features

- Configures and initializes the MAX31723 temperature sensor
- Reads temperature data via SPI communication
- Outputs formatted temperature readings via serial at 115200 baud
- Provides visual status indication for computation and SPI communication
- 12-bit temperature resolution

## Code Functionality

The main application:
- Initializes the IOPORT for pin configuration
- Sets up serial communication at 115200 baud rate
- Configures the MAX31723 sensor with 12-bit resolution via SPI
- Continuously reads temperature data every second
- Displays status information and temperature readings via serial
- Implements error handling for SPI communication failures

## Getting Started

### Setup

1. Connect the MAX31723 sensor to the Ra2E1 Feather board's SPI pins
2. Connect the CS pin to P104 (Slave Select)
3. Import the project into e² studio
4. Build and flash the project to the Ra2E1 Feather board
5. Connect to the serial port at 115200 baud to view temperature readings

### Configuration

The MAX31723 sensor is configured with 12-bit resolution:

```c
/* Value for configuration register write. Set resolution as 12-bits*/
const uint8_t config_sensor[3] =
{
 0x80, 0x06
};
```

## Project Structure

- `src/hal_entry.cpp`: Main application entry point and loop
- `src/spi_max31723.cpp`: Implementation of the SPI communication with MAX31723
- `src/spi_max31723.h`: Header file with MAX31723 interface definitions
- `src/SerialCompatibility.cpp/.h`: Serial communication utilities
- `src/common_utils.h`: Common utility functions

## License

Copyright © 2023 Zalmotek

## Additional Resources

- [Zalmotek Website](https://zalmotek.com)
- [Zalmotek RA2E1 Website](https://zalmotek.com/products/RA2E1-Feather-SoM/)
- [SEGGER RTT Documentation](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) 