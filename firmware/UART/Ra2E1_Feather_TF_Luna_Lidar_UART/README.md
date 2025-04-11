# Zalmotek Ra2E1 Feather TF Luna Lidar UART

A Zalmotek project using UART communication to interface a TF Luna Lidar sensor with Renesas Ra2E1 microcontroller.

## Overview

This project demonstrates how to connect and read distance data from a TF Luna Lidar sensor using the Zalmotek Ra2E1 Feather board powered by Renesas RA2E1 microcontroller. The application uses UART communication to receive distance and signal strength readings from the sensor.

## Hardware Requirements

- Zalmotek Ra2E1 Feather board 
- TF Luna Lidar sensor
- Connection cables
- USB cable for power and programming

## Software Requirements

- Renesas e² studio IDE
- Renesas FSP (Flexible Software Package)
- J-Link debugging tools

## Features

- Real-time distance measurement using TF Luna Lidar
- Signal strength monitoring
- UART serial communication interface
- Debugging output via SEGGER RTT

## Code Functionality

The main application:
- Initializes UART communication at the appropriate baud rate for the TF Luna
- Continuously reads 9-byte data frames from the TF Luna sensor
- Parses the data frames to extract distance (in cm) and signal strength values
- Outputs the readings to debug console
- Provides Serial compatibility layer for simplified UART communication

## Getting Started

### Setup

1. Connect the TF Luna Lidar sensor to the UART pins on the Zalmotek Ra2E1 Feather board
2. Import the project into Renesas e² studio
3. Build the project using the Debug configuration
4. Flash the binary to the Ra2E1 Feather board using J-Link
5. Monitor the output using SEGGER RTT Viewer or serial terminal

### Configuration

The UART is configured to read data from the TF Luna sensor that communicates using a standard protocol:

```c
// TF Luna protocol parsing example
if (buf[0] == 0x59 && buf[1] == 0x59) {
    distance = (uint16_t)(buf[2] + buf[3] * 256);
    strength = (uint16_t)(buf[4] + buf[5] * 256);
}
```

## Project Structure

- `src/hal_entry.cpp`: Main application entry point and TF Luna reading code
- `src/SerialCompatibility.cpp`: Arduino-style serial interface implementation
- `src/SerialCompatibility.h`: Header for the serial compatibility layer
- `src/SEGGER_RTT/`: SEGGER Real-Time Transfer for debugging output

## License

Proprietary - Zalmotek

## Additional Resources

- [Zalmotek Website](https://zalmotek.com)
- [Zalmotek RA2E1 Website](https://zalmotek.com/products/RA2E1-Feather-SoM/)
- [SEGGER RTT Documentation](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) 