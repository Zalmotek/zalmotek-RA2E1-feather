# Zalmotek RA2E1 Feather USB Serial

A UART communication example for the Zalmotek RA2E1 Feather board powered by Renesas RA2E1 microcontroller.

## Overview

This project demonstrates USB serial communication capabilities of the Zalmotek RA2E1 Feather board. It establishes a serial connection over USB, allowing data exchange between the board and a computer terminal. The implementation uses Renesas RA2E1 microcontroller's USB functionality.

## Hardware Requirements

- Zalmotek RA2E1 Feather board
- USB Type-C cable
- Computer with terminal software

## Software Requirements

- Renesas e² studio IDE
- FSP (Flexible Software Package)
- Terminal software (PuTTY, Tera Term, or similar)

## Features

- USB serial communication
- Simple command interface
- Status LED indication
- Configurable baud rate

## Code Functionality

The main application:
- Initializes the USB device controller
- Sets up UART communication parameters
- Processes incoming serial data
- Provides basic command response capabilities
- Controls onboard LED based on commands

## Getting Started

### Setup

1. Connect the Zalmotek RA2E1 Feather board to your computer using a USB-C cable
2. Import the project into e² studio
3. Build and flash the firmware to the board
4. Open a terminal program on your computer
5. Connect to the board's COM port with default settings (115200 baud, 8N1)

### Configuration

The USB serial settings can be modified in the configuration:

```c
// Example USB serial configuration
#define BAUD_RATE       115200
#define DATA_BITS       8
#define PARITY          UART_PARITY_OFF
#define STOP_BITS       UART_STOP_BITS_1
```

## Project Structure

- `src/`: Contains source code files
- `ra/`: Renesas FSP configuration and drivers
- `ra_gen/`: Auto-generated code from FSP configurator
- `ra_cfg/`: User configuration files

## License

Proprietary - Copyright © 2023 Zalmotek

## Additional Resources

- [Zalmotek Website](https://zalmotek.com)
- [Zalmotek RA2E1 Website](https://zalmotek.com/products/RA2E1-Feather-SoM/)
- [SEGGER RTT Documentation](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) 