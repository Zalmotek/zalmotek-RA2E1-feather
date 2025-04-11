# Zalmotek RA2E1 Feather ADC

A Zalmotek RA2E1 Feather board based analog-to-digital converter (ADC) application powered by Renesas RA2E1 microcontroller.

## Overview

This project demonstrates the ADC functionality of the Zalmotek RA2E1 Feather board, featuring Renesas RA2E1 microcontroller. The application initializes the ADC in single scan or continuous scan mode based on user selection, reads analog values, converts them to digital values, and displays the results via SEGGER RTT Viewer.

## Hardware Requirements

- Zalmotek RA2E1 Feather board 
- JLink debugger
- PC with USB connection

## Software Requirements

- e² studio IDE
- FSP (Flexible Software Package)
- JLink RTT Viewer
- GCC ARM Embedded toolchain

## Features

- Configurable ADC scan mode (single or continuous)
- User interaction through JLink RTT Viewer
- ADC channel reading with voltage conversion display
- Window comparator functionality
- ADC calibration capability (for supported boards)

## Code Functionality

The main application:
- Initializes the ADC module with user-selected parameters
- Configures GPIO pins for analog readings
- Provides commands to start/stop ADC scans via RTT Viewer
- Reads multiple ADC channels and converts values to voltages
- Supports window comparator functionality for threshold detection
- Handles ADC calibration on supported hardware

## Getting Started

### Setup

1. Connect the Zalmotek RA2E1 Feather board to your PC
2. Open the project in e² studio
3. Build the project
4. Connect J-Link debugger and flash the application to the board
5. Open JLink RTT Viewer to interact with the application

### Configuration

Use the RTT Viewer to control the application with these commands:

```
Press 1 to Start ADC Scan
Press 2 to Stop ADC Scan (Only for Continuous mode)
```

## Project Structure

- `src/adc_ep.c`: Main ADC functionality implementation
- `src/adc_ep.h`: Header file with ADC-related definitions
- `src/hal_entry.c`: Entry point and main application loop
- `src/SEGGER_RTT/`: RTT communication utilities
- `src/common_utils.h`: Common utility functions and definitions

## License

SPDX-License-Identifier: BSD-3-Clause

## Additional Resources

- [Zalmotek Website](https://zalmotek.com)
- [Zalmotek RA2E1 Website](https://zalmotek.com/products/RA2E1-Feather-SoM/)
- [SEGGER RTT Documentation](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) 