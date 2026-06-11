@mainpage ME507 Marshmallow Cooker

# Project Overview

The ME507 Marshmallow Cooker is an automated marshmallow roasting system designed to monitor temperature and control marshmallow position and orientation during cooking.

<p align="center">
  <img src="../images/intro.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

The system uses both infrared and thermocouple-based temperature measurements to perfect cooking conditions while a rotating mechanism and vertical positioning mechanism control the marshmallow relative to the heat source.

---

# Major Hardware

## Rotating Motor Assembly

The rotating axis uses a Pololu 5120 gearmotor with integrated quadrature encoder. The 297.92:1, 45 rpm max motor provides slow, yet steady rotation of the marshmallow. A DRV8833 motor driver at 5V allows bidirectional PWM control from the STM32 microcontroller. A simple software driver was implemented to abstract direction control and PWM duty cycle commands.

<p align="center">
  <img src="../images/r_motor.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

The 12 CPR quadrature encoder provides position feedback that allows the software to estimate angular position and rotational velocity. Hardware timers count the encoder ticks, measuring the angular displacement of the motor.

<p align="center">
  <img src="../images/r_motor_schematic.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

## Vertical Motion System

<p align="center">
  <img src="../images/z_motor.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

The distance between the marshmallow and the heat source is adjusted by a lead screw system powered by a stepper motor, DFRobot FIT0278 stepper motor. The motor is controlled by a TMC2209 driver at 12V. The FIT0278 has 200 steps per revolution, and the motor driver is configured for 8 microsteps per step. The result is smooth, fine control of the height of the marshmallow above the fire.

The TMC2209 stepper motor driver schematic is shown below. We installed a potentiometer into the resistor divider for VREF, so we could tune the maximum current sent to the stepper motor. Our VREF range is roughly 0.8V to 2.5V. Additionally, we sized shunt resistor for the driver to sense the current.

<p align="center">
  <img src="../images/z_motor_schematic.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

Top and bottom limit switches are used to prevent the mechanism from exceeding its allowable travel range. The limit switches are wired normally closed (NC), with internal pull up resistors configured on the STM32. When a limit switch is pressed, the ground connection is broken, and the pin is forced high by the pull up resistor. This means that a limit switch failure, or unplugging the pins. This fail-safe behavior causes a disconnected or damaged switch to be detected as a triggered limit condition.

## Temperature Sensors

### MLX90614 Infrared Temperature Sensor

The MLX90614 provides non-contact temperature measurements of the marshmallow surface. This allows surface temperature to be monitored without physically touching the marshmallow. When the marshmallows surface reaches a set temperature, the cook is done.

<p align="center">
  <img src="../images/ir_temp.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

The MLX90614 is an integrated IR temperature camera and I2C device. This makes implementation very simple as it does not require an amplifier or driver. The IR temperature sensor itself communicates with the STM32 via I2C CLK and SDA.

### MCP9600 Thermocouple Amplifier

The MCP9600 thermocouple amplifier interfaces with a Type T thermocouple to provide direct hot-junction temperature measurements of the flame. The flame temperature is used in our control loop to adjust the height of the marshmallow.

<p align="center">
  <img src="../images/thermocouple.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

The MCP9600 is also an I2C device. It has a configurable address pin, which is grounded, setting the address to 0x60. The hot junction temperature is read via the hardware I2C bus as the IR temperature sensor.

## Custom PCB

<p align="center">
  <img src="../images/pcb.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

A custom PCB was designed to integrate all hardware components:

* STM32F411 microcontroller
* MCP9600 thermocouple amplifier
* MLX90614 infrared temperature sensor
* DRV8833 motor driver
* \ref REncoderDriver "Encoder" inputs
* TMC2209 stepper motor driver interface
* \ref ZLimitSwitches "Limit switch" inputs

<p align="center">
  <img src="../images/pcb_fusion_combined.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

The PCB simplifies wiring and provides a compact platform for system integration. Our board begins with the 12 VDC barrel plug, designed for an external wall AC to DC converter. The 12 VIN goes through a fuse (MFU0805FF03000P500) and p-fet reverse polarity circuit (SI4435FDY-T1-GE3). Then, a high efficient switching buck regulator (MP2338GTL-Z) drops the voltage to 5V for the rotating motor. Finally, a linear dropout regulator (LDL1117S33R) reduces the voltage to 3.3V, our digital logic level (VDD).

Key PCB Design Choices:
* Internal planes for GND and VDD
* Surface mounted components only on the top layer
* Reducing high power trace lengths
* Potentiometer for TMC2209 VREF
* Pads for optional stepper motor phase decoupling capacitors

Due to the project's tight timeline, we found multiple issues with our PCB that we would fix if given time and money for another board revision.

Issues We Encountered:
* Wrong pins for I2C
  * Problem: We mixed up ports A and B in the schematic. Connected traces to PA6 & PA7 instead of PB6 & PB7 for I2C1.
  * Solution: Soldered 32 AWG Kynar wire to jump pins PA8 & PB8 (I2C3) to the pull up resistor and rest of the I2C pcb wiring.
* Switching regulator placed 180 degrees
  * Problem: When purchasing the PCB we probably missed seeing that the switching regulator was placed 180 degrees incorrectly.
  * Solution: Used hot air soldering gun to remove the switching regulator, rotate it 180 degrees, and place it back on the pads.
* Thru holes for stepper motor JST connector too small
  * Problem: Did not verify the via size of the component we downloaded. Result was vias were too small for the thru hole pins of the JST-XH connector.
  * Solution: Attempted to expand the vias, but lifted the pads and some traces. Cut back the lifted traces, chipped off the solder mask to expose the copper, soldered the connector's pins onto the copper traces. Super glued the connector's body onto the board. Never unplugging the connector on the board side.

<p align="center">
  <img src="../images/back.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

---

# CAD

The GitHub repository contains CAD for the marshmallow cooker. This includes a main assembly file, individual part models, and hardware (screws and nuts).

<p align="center">
  <img src="../images/CAD_1.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

<p align="center">
  <img src="../images/CAD_2.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

<p align="center">
  <img src="../images/CAD_3.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

<p align="center">
  <img src="../images/CAD_4.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

---

# Software Architecture

The firmware is written in C++ using STM32CubeIDE and STM32 HAL.

Major software modules include:

* DRV8833 motor driver
* MCP9600 thermocouple driver
* MLX90614 infrared sensor driver
* Encoder driver
* Rotational motor driver
* Z-axis motor driver
* Z-axis limit switch driver

Hardware-specific functionality is encapsulated within dedicated driver classes, allowing higher-level control logic to remain independent of hardware implementation details.

---

# Main Control Structure

TODO

---

# User Interface

TODO

---

# Control Loop

Todo

---

# Repository Structure

```text
ME507MarshmallowCooker
├── Marshmallow_Cooker
│   ├── Core
│   └── Drivers
├── docs
└── Doxyfile
```

## Repository

GitHub Repository:

[Link back to ME507MarshmallowCooker GitHub Repository](https://github.com/CaidenBonney/ME507MarshmallowCooker)