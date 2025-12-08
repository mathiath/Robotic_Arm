# Robotic Arm – HiØ Anvendt Robotteknikk (Fall 2025)

Firmware and serial communication interface for a 6-DOF robotic arm developed for the Applied Robotics course at Høgskolen i Østfold.

This code runs on a Teensy 4.1 and communicates with RoboDK through a Python bridge.

---

## Installing the Firmware
1. Install Arduino IDE.
2. Use the following instructions to add the Teensy 4.1 board to Arduino IDE: https://www.pjrc.com/teensy/td_download.html
3. Open `main.ino` in the Arduino IDE.
4. Select **Teensy 4.1** under Tools - Board.
5. Upload.

The file `serial.ino` is automatically detected and compiled by Arduino.

---

## Basic Usage

### 1. Startup Procedure

To ensure correct homing and encoder alignment:

1. Place the robot in the upright zero position (all joints at 0°)
2. Insert zeroing pins
3. Power the motor drivers
4. Connect USB to the Teensy, and wait for it to initialize and calibrate
5. Remove the pins before any movement is executed

### 2. Home Button

Pressing the HOME button moves the robot to the zero position for all joints.

### 3. FreeMove Mode

- The FreeMove button toggles the motor power.
- When FreeMove is active:
  - Motors are disabled.
  - The arm can be moved freely by hand, while still sending encoder-data to RoboDK

### 4. Serial Control

The controller accepts the following commands:

- `R[...]`  – set joint targets  
- `L<number>` – set robot speed  
- `E` – return current joint values  
- `G0` / `G1` – gripper off/on

---

## Emergency Stop (E-STOP)

- Pressing the E-STOP button immediately disables all motors. 
- The arm may fall freely depending on payload and joint position.

The E-STOP is a latching, dual-circuit safety stop:
- One circuit physically cuts the stepper driver ENABLE lines.
- The other provides a firmware-level stop for redundancy, and allows for future digital feedback to RoboDK.

The arm will not be operational until the E-STOP is manually unlatched and the system has been power-cycled and re-homed.

### Reset Procedure
1. Unplug the power to the arm and disconnect the USB cable
2. Unlatch the E-STOP button
3. Place the robot in the upright zero position
4. Insert zeroing pins
5. Reconnect motor power
6. Reconnect USB to the Teensy and wait for it to initialize and calibrate
7. Remove the pins before any movement is executed

---

## File Overview

- `main.ino` - Firmware: stepping, encoders, safety, button handling
- `serial.ino` - Serial parser used for RoboDK/Python communication 

Both files are compiled automatically by the Arduino IDE.

---

## Notes

- Current setup uses a single shared speed value for all joints, due to the RoboDK - Python interface.
- Acceleration smoothing is not implemented in this version.
