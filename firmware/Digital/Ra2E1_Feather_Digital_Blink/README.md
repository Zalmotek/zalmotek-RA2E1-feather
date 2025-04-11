# Zalmotek RA2E1 Feather Blink

A simple LED blink example for the Zalmotek RA2E1 Feather board powered by Renesas RA2E1 microcontroller.

## Overview

This project demonstrates the basic functionality of the Zalmotek RA2E1 Feather board by blinking the onboard LEDs. It showcases simple GPIO control using the Renesas RA2E1 microcontroller, with the LEDs toggling at a 200ms interval.

## Hardware Requirements

- Zalmotek RA2E1 Feather board 
- USB cable for power and programming

## Software Requirements

- Renesas e² studio IDE
- Renesas FSP (Flexible Software Package)
- J-Link debugger software

## Features

- Simple LED blinking demonstration
- Utilizes Renesas RA FSP for GPIO control
- 200ms toggle interval for clear visual feedback
- SEGGER RTT support for debug output

## Code Functionality

The main application:
- Initializes the I/O ports using R_IOPORT_Open
- Enables pin access with R_BSP_PinAccessEnable
- Toggles all onboard LEDs between high and low states
- Uses a 200ms delay between state changes
- Runs in a continuous loop

## Getting Started

### Setup

1. Clone or download this repository
2. Open e² studio IDE
3. Import the project into your workspace
4. Build the project
5. Connect the RA2E1 Feather board via USB
6. Program the board using the J-Link debugger

### Configuration

The LED blink rate can be modified by changing the delay value:

```c
// Change this value to adjust blink speed (in milliseconds)
R_BSP_SoftwareDelay(200, bsp_delay_units);
```

## Project Structure

- `src/hal_entry.cpp`: Main application code with LED blinking implementation
- `src/common_utils.h`: Common utilities and definitions for the project
- `src/SEGGER_RTT/`: SEGGER RTT implementation for debug output
- `.settings/`: Project configuration files
- `configuration.xml`: FSP configuration file

## License

Renesas Electronics Corporation Proprietary

## Additional Resources

- [Zalmotek Website](https://zalmotek.com)
- [Zalmotek RA2E1 Website](https://zalmotek.com/products/RA2E1-Feather-SoM/)
- [SEGGER RTT Documentation](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) 