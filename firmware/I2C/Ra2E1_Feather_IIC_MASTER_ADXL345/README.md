# Zalmotek RA2E1 Feather I2C Master ADXL345

I2C Master implementation for Zalmotek RA2E1 Feather board powered by Renesas, communicating with an ADXL345 accelerometer.

## Overview

This project demonstrates I2C communication between a Zalmotek RA2E1 Feather board featuring Renesas microcontroller technology and an ADXL345 accelerometer. It continuously reads acceleration data from the sensor and displays the X, Y, and Z axis measurements via serial output.

## Hardware Requirements

- Zalmotek RA2E1 Feather board 
- ADXL345 accelerometer
- Connection cables

## Software Requirements

- e² studio IDE
- FSP (Flexible Software Package)
- Serial terminal software (115200 baud)

## Features

- I2C master communication with ADXL345 accelerometer
- Real-time acceleration data acquisition (X, Y, Z axes)
- Serial output of sensor readings
- Sensor initialization and continuous monitoring

## Code Functionality

The main application:
- Initializes the I2C interface to communicate with the ADXL345 accelerometer
- Configures the ADXL345 by enabling measurement mode
- Continuously reads acceleration data on all three axes
- Processes raw sensor data into X, Y, Z values
- Displays formatted acceleration data via serial output at 1-second intervals
- Includes error handling for I2C communication failures

## Getting Started

### Setup

1. Connect the ADXL345 accelerometer to the RA2E1 Feather board's I2C pins
2. Import the project into e² studio
3. Build the project
4. Flash the binary to the RA2E1 Feather board
5. Open a serial terminal (115200 baud rate)
6. Move the accelerometer to see changing X, Y, Z values

### Configuration

The ADXL345 sensor is configured in the initialization function:

```c
void init_sensor(void)
{
    // Configure the ADXL345 to enable measurements
    uint8_t measure_enable_payload[MEASURE_PAYLOAD_SIZE] = { POWER_CTL_REG, ENABLE_BIT };
    R_IIC_MASTER_Write(&g_i2c_master0_ctrl, measure_enable_payload, TWO_BYTE, false);
}
```

## Project Structure

- `src/hal_entry.cpp`: Main application entry point and continuous sensor reading loop
- `src/i2c_sensor.cpp`: I2C communication implementation for the ADXL345 accelerometer
- `src/i2c_sensor.h`: Constants and function declarations for sensor operations
- `src/SerialCompatibility.cpp`: Serial communication wrapper functions
- `src/SerialCompatibility.h`: Serial interface declarations

## License

This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. See included license information in source files.

## Additional Resources

- [Zalmotek Website](https://zalmotek.com)
- [Zalmotek RA2E1 Website](https://zalmotek.com/products/RA2E1-Feather-SoM/)
- [SEGGER RTT Documentation](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) 