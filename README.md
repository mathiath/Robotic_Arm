# Robotic Arm – HiØ Anvendt Robotteknikk (Fall 2025)

Firmware and serial communication interface for a 6-DOF robotic arm developed for the Applied Robotics course at Høgskolen i Østfold.

This code runs on a Teensy 4.1 and communicates with RoboDK through a Python bridge.

---

## Installing the Firmware

1. Open `main.ino` in the Arduino IDE.
2. Select **Teensy 4.1** under Tools - Board.
3. Upload.

The file `serial.ino` is automatically detected and compiled by Arduino.

---

## Basic Usage

### 1. Startup Procedure

To ensure correct homing and encoder alignment:

- Place the robot in the upright zero position (all joints at 0°).
- Insert zeroing pins.
- Power the motor drivers.
- Connect USB to the Teensy, and wait for it to initialize and calibrate.
- Remove the pins before any movement is executed.

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

- Pressing the E-STOP button immediately disables the motors. The arm may fall freely depending on load and position.

### Resetting E-STOP

E-STOP requires a full power cycle and re-homing of the system.

- Unplug the power to the arm and the USB.
- Place the robot in the upright zero position (all joints at 0°).
- Insert zeroing pins.
- Power the motor drivers.
- Connect USB to the Teensy, and wait for it to initialize and calibrate.
- Remove the pins before any movement is executed.

---

## File Overview

- `main.ino` - Firmware: stepping, encoders, safety, button handling
- `serial.ino` - Serial parser used for RoboDK/Python communication 

Both files are compiled automatically by the Arduino IDE.

---

## Notes

- Current setup uses a single shared speed value for all joints due to the RoboDK - Python interface.
- Acceleration smoothing is not implemented in this version.
