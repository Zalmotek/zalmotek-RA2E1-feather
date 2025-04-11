# Zalmotek RA2E1 Feather Digital Blink

A digital pin blinking demonstration for the Zalmotek RA2E1 Feather board powered by Renesas RA2E1 microcontroller.

## Overview

This project demonstrates sequential blinking of digital pins on the Zalmotek RA2E1 Feather board. It cycles through pins D5, D6, D7, D9, D10, D11, D12, and D13, turning each on and off with a 200ms delay between state changes.

## Hardware Requirements

- Zalmotek RA2E1 Feather board 
- USB cable for power and programming
- Optional: LEDs and resistors connected to digital pins

## Software Requirements

- Renesas e² studio IDE
- FSP (Flexible Software Package)
- Renesas RA2E1 toolchain

## Features

- Sequential digital pin control
- Configurable delay between pin state changes
- Detailed pin state feedback via debug console
- Utilizes Renesas FSP for hardware abstraction

## Code Functionality

The main application:
- Initializes GPIO pins (D5, D6, D7, D9, D10, D11, D12, D13)
- Sequentially toggles each pin from LOW to HIGH
- Implements a 200ms delay between state changes
- Operates in a continuous loop mode
- Outputs pin state changes via debug console

## Getting Started

### Setup

1. Connect the Zalmotek RA2E1 Feather board to your computer via USB
2. Open the project in Renesas e² studio
3. Build the project
4. Flash the program to the board
5. Observe the sequential blinking pattern on the pins

### Configuration

To modify the delay between pin state changes:

```c
// Change the delay value (in milliseconds)
delay(200);
```

## Project Structure

- `src/hal_entry.cpp`: Main application code with pin control logic
- `src/common_utils.h`: Utility functions and definitions
- `src/SEGGER_RTT/`: Debug output functionality
- `.settings/`: IDE configuration files
- `script/`: Build and flash scripts

## License

[License information]

## Additional Resources

- [Zalmotek Website](https://zalmotek.com)
- [Zalmotek RA2E1 Website](https://zalmotek.com/products/RA2E1-Feather-SoM/)
- [SEGGER RTT Documentation](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) 
