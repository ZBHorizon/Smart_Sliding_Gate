# Smart_Sliding_Gate

**A project to control and automate a sliding gate with a Raspberry Pi (using wiringPi), including features like soft motor ramping, end-switch detection, and future Smart Home integration.**

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Hardware Requirements](#hardware-requirements)

---

## Overview

Smart_Sliding_Gate is a C++ based application designed for controlling a sliding gate using a Raspberry Pi. 
It controls a motor (via PWM output and a direction pin), reads end-stop switches to prevent overtravel, 
and allows for **soft** acceleration/deceleration. 

In the future, it will be extended to support **position control**, 
**Smart Home integration** (e.g., via MQTT, Home Assistant, or custom APIs), 
and potentially sensor-based obstacle detection.

---

## Features

- **Soft Start/Stop**: Gradual ramp-up/down of motor speed to reduce mechanical stress.
- **End Stops**: Automatic detection of left/right end switches, halting the motor if triggered.
- **Button Press Logic**: Simple open/close toggles, including logic for toggling between open/stop/close.
- **WiringPi** usage for GPIO and hardware PWM.
- Smart Home integration (MQTT, etc.)
- Position control (e.g., partial opening).
- Obstacle detection or safety sensor inputs.
...

---

## Hardware Requirements

- **Raspberry Pi** (tested with e.g. Raspberry Pi 3/4; wiringPi installed).
- **Motor driver board** (supports PWM + direction control).
- **Sliding gate motor** rated for the driver board.
- **Two end-stop switches** for detecting full open/close positions.
- **Buttons** for manual open/close (optional).
- **Power supply** capable of handling motor current peaks.



