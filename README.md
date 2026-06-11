# ME507 Marshmallow Cooker

## Overview

The ME507 Marshmallow Cooker is an automated system designed to cook the perfect marshmallow for s'mores! The marshmallow cooker roasts the marshmallows to a desired temperature using sensor feedback and motorized positioning.

The system is based around a STM32F411 microcontroller and incorporates both contact and non-contact temperature sensing. The rotating mechanism (rotisserie) ensures an even cook while the vertical positioning system modulates the distance between the marshmallow and the heat source (demoing with a Sterno).

## Team Members

* Caiden Bonney
* Toby Sagi

## Hardware

### Sensors

* MLX90614 infrared temperature sensor
* MCP9600 thermocouple amplifier with Type T thermocouple
* Top and bottom Z-axis limit switches

### Actuators

* Pololu 5120 gearmotor with quadrature encoder with DRV8833 brushed DC motor driver
* DF Robot Fit0278 stepper motor with TMC2209 stepper motor driver

### Custom Electronics

* Custom PCB designed for STM32, sensor integration, and motor control.

## Firmware Architecture

The firmware is written primarily in C++ using STM32CubeIDE and STM32 HAL.

Major drivers include:

* Task_UI
* Task_Temps
* Task_R_Motor
* Task_Z_Motor
* MCP9600 thermocouple driver
* MLX90614 infrared temperature driver
* Encoder driver
* DRV8833 motor driver
* Rotational motor driver
* Z-axis limit switch driver
* DRV8833 motor driver
* Z-axis motor driver

## Documentation

Project documentation:

[ME507 Marshmallow Cooker Documentation](https://caidenbonney.github.io/ME507MarshmallowCooker/)

## Repository Structure

```text
ME507MarshmallowCooker
├── CAD
├── docs
├── Fusion
├── Marshmallow_Cooker
│   ├── Core
│   └── Drivers
├── Doxyfile
└── README

```

## Building the Project

1. Open `Marshmallow_Cooker` in STM32CubeIDE.
2. Build the project.
3. Flash the firmware to the STM32F411 target.
4. Open a serial terminal at 115200 baud for debugging output.

## License

This repository was created for ME507 at California Polytechnic State University.
