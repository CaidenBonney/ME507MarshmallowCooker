@mainpage ME507 Marshmallow Cooker

# Project Overview

The ME507 Marshmallow Cooker is an automated marshmallow roasting system designed to monitor temperature and control marshmallow position and orientation during cooking.

<p align="center">
  <img src="../images/main.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

The system uses both infrared and thermocouple-based temperature measurements to perfect cooking conditions while a rotating mechanism and vertical positioning mechanism control the marshmallow relative to the heat source.

## Video Demonstration

[Click here to watch the Marshmallow Cooker in action!](https://cpslo-my.sharepoint.com/:v:/g/personal/clbonney_calpoly_edu/IQBmmSWm59pES4WM_rZA4jZBAax7FDM9BySxYHA97Abe1Qw?e=9FayDm)

---

# Major Hardware

## BOM

Here is a link to our [Bill of Materials](https://cpslo-my.sharepoint.com/:x:/g/personal/clbonney_calpoly_edu/IQCO9qkZxfa1SrjVX5YEZTvCAYXNW5hFe5g4LbXm4rrSlMY?e=KhwhCP) (does not include CAD components)

## Rotating Motor Assembly

The rotating axis uses a Pololu 5120 gearmotor with integrated quadrature encoder. The 297.92:1, 45 rpm max motor provides slow, yet steady rotation of the marshmallow. A DRV8833 motor driver at 5V allows bidirectional PWM control from the STM32 microcontroller. A simple software driver was implemented to abstract direction control and PWM duty cycle commands.

<p align="center">
  <img src="../images/2_moving.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

The 12 CPR quadrature encoder provides position feedback that allows the software to estimate angular position and rotational velocity. Hardware timers count the encoder ticks, measuring the angular displacement of the motor.

<p align="center">
  <img src="../images/r_motor_schematic.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

## Vertical Motion System

<p align="center">
  <img src="../images/1_front.jpg"
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

<p align="center">
  <img src="../images/3_tc_temp.jpg"
       style="max-width:100%; max-height:400px; width:auto; height:auto;">
</p>

### MLX90614 Infrared Temperature Sensor

The MLX90614 provides non-contact temperature measurements of the marshmallow surface. This allows surface temperature to be monitored without physically touching the marshmallow. When the marshmallows surface reaches a set temperature, the cook is done.

The MLX90614 is an integrated IR temperature camera and I2C device. This makes implementation very simple as it does not require an amplifier or driver. The IR temperature sensor itself communicates with the STM32 via I2C CLK and SDA.

### MCP9600 Thermocouple Amplifier

The MCP9600 thermocouple amplifier interfaces with a Type T thermocouple to provide direct hot-junction temperature measurements of the flame. The flame temperature is used in our control loop to adjust the height of the marshmallow.

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
  <img src="../images/4_back.jpg"
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

The application uses a cooperative task structure built around the shared \ref Task interface. After STM32Cube-generated initialization configures GPIO, USART2, I2C3, TIM1 PWM, TIM3 encoder mode, and USB device support, `main()` repeatedly calls the application tasks in a fixed order:

* \ref TaskUI "TaskUI" parses any completed user command.
* \ref TaskTemps "TaskTemps" periodically updates the thermocouple and IR temperature readings.
* \ref TaskCooker "TaskCooker" consumes commands, checks sensor data, and issues high-level motion requests.
* \ref TaskRMotor "TaskRMotor" updates the rotisserie motor.
* \ref TaskZMotor "TaskZMotor" updates the vertical stepper motor.

This ordering lets new command and temperature data be processed before the motor tasks act on the latest cooker decisions. The tasks are written as small non-blocking state machines using `HAL_GetTick()` timing instead of long blocking delays. The temperature and Z-control tasks run their scheduled logic every 500 ms, while the R motor task updates every 10 ms so encoder-driven rotation remains responsive.

---

# User Interface

The firmware provides a serial command-line interface through \ref TaskUI "TaskUI". USART2 is configured for 115200 baud, 8 data bits, no parity, and 1 stop bit. The UI task receives one byte at a time with `HAL_UART_Receive_IT()`, stores characters in a small interrupt-to-task queue, echoes printable input, supports backspace, and parses commands when the user presses Enter.

The user-facing cook sequence is coordinated by \ref TaskCooker "TaskCooker". On startup it waits for the `home` command, then commands \ref TaskZMotor "TaskZMotor" to home upward until the top limit switch defines Z = 0. Once homed, the cooker enters `ReadyToCook` and accepts `start`. Starting a cook moves Z to the initial cooking height, enables flame-temperature control, and then starts the R-axis rotisserie once Z reaches active control. A normal stop, completed cook, or bottom-limit condition commands the R motor back to its initial rotation and moves Z to the removal height. Faults and emergency stops immediately stop both motor tasks and require `reset` followed by another `home` before cooking again.

Supported commands are:

* `home` - home the Z axis against the top limit switch and prepare the cooker.
* `start` - begin a cooking cycle after Z has been homed.
* `stop` - perform a normal stop by returning R to zero and moving Z to removal height.
* `estop` - immediately enter the fault state and stop both motors.
* `reset` - clear a `Fault` or `Done` state and require another home cycle.
* `status` - print one status line.
* `status <ms>` - stream status lines for the requested duration in milliseconds.
* `rotate` - toggle manual R-axis cooking rotation while ready.
* `piddebug on` / `piddebug off` - enable or disable Z PID debug messages.
* `- <steps>` - jog Z downward by the requested number of steps.
* `= <steps>` - jog Z upward by the requested number of steps.

If no step count is supplied for `-` or `=`, the default jog distance is 100 steps. Status output reports the cooker state, Z task state, R task state, Z position, top and bottom limit switch states, thermocouple hot-junction temperature, and IR object temperature. `print_str()` sends firmware messages over USART2 and also mirrors them to USB CDC when USB support is present in the build.

---

# Control Loop

The cooking loop uses two temperature measurements for two different jobs. \ref TaskTemps "TaskTemps" polls the MCP9600 thermocouple amplifier and MLX90614 IR sensor over I2C3 every 500 ms. The thermocouple hot-junction reading is treated as the flame temperature used by Z-axis control, while the IR object reading is used as the marshmallow done-temperature measurement.

When `start` is accepted, \ref TaskCooker "TaskCooker" commands \ref TaskZMotor "TaskZMotor" to move from the homed top reference to the starting cook position at -11000 steps. Once that move finishes, Z enters `ControllingFlameTemp`. The cooker then starts the R-axis cooking motion, which alternates +360 degree and -360 degree moves so the marshmallow rotates without winding wires indefinitely. Since writing code, the mechanical system has changed and this is no longer necessary. The motor could spin in one direction indefinitely.

The Z-axis PID loop runs every 500 ms with a target flame temperature of 240.00 F. Temperatures are stored as fixed-point Fahrenheit values multiplied by 100. The PID error is:

```text
error = target flame temperature - measured flame temperature
```

A positive error means the measured flame temperature is too cold, so the Z target is moved downward toward the flame. A negative error means the flame is too hot, so the Z target is moved upward. The default gains are `Kp = 1.0`, `Ki = 0.02`, and `Kd = 0.10`. Each PID update is clamped to +/-750 steps with a +/-100 step deadband, and the integral term is limited to reduce wind-up. Software limits prevent commands above the home position or below the -16000 step lower cook limit.

Cooking completes when the IR object temperature reaches 250.00 F continuously for 1.5 second. At that point the cooker begins a normal stop: the R motor returns to encoder zero and Z moves to the -500 step removal height. If a motor fault or emergency stop occurs, the cooker enters `Fault`, stops motion, and waits for a reset sequence.

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
