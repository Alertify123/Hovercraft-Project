# Autonomous Maze-Navigating Hovercraft

Bare-metal embedded robotics project built on an **ATmega328P** in C, featuring **PID yaw stabilization**, **MPU-6050 IMU integration**, and autonomous maze navigation using IR + ultrasonic sensing.

---

## Team
- Chrisjan Alejandro  
- Juan Sebastian Holguin Corpas  
- Mcwill Buikpor  
- Niraj Patel  
- Philippe Hadley Plancher

---

## Highlights
- Bare-metal AVR firmware (no Arduino libraries)
- Real-time heading stabilization with PID + gyro feedback
- Multi-sensor perception stack: IR wall sensing + ultrasonic upbar detection
- Autonomous decision-making: straight driving, intersection scanning, dead-end recovery, finish stop
- Modular firmware layout (`src/`, `include/`, `config.h`) for maintainability

---

## Repository Structure

```text
Hovercraft-Project/
├── Makefile
├── config.h
├── include/
│   ├── imu.h
│   ├── motion.h
│   ├── pid.h
│   ├── sensors.h
│   └── system.h
├── src/
│   ├── imu.c
│   ├── main.c
│   ├── motion.c
│   ├── pid.c
│   └── sensors.c
└── legacy/
    └── FinalHovercraftCode_290_TEAM3_FALL_2025.c
```

---

## Control Architecture

### Main loop
1. `drive_straight_until_wall()`
2. `handle_intersection()`
3. repeat

### Key modules
- **`src/main.c`**: 1 kHz systick ISR, software PWM timing, boot sequence, main navigation loop
- **`src/imu.c`**: TWI/I2C driver, MPU setup, gyro calibration, yaw integration
- **`src/sensors.c`**: ADC + IR distance conversion, HC-SR04 trigger/echo timing, upbar confirmation logic
- **`src/pid.c`**: Yaw PID controller (anti-windup + filtered derivative)
- **`src/motion.c`**: Servo/fan actuation, turn maneuvers, straight driving, intersection solver

---

## Build

### Requirements
- `avr-gcc`
- `avr-objcopy`
- `avr-size`

### Commands
```bash
make        # build hovercraft.elf + hovercraft.hex
make size   # AVR memory usage
make clean  # remove build artifacts
```

---

## Tuning
All tunable constants are centralized in `config.h`, including:
- Pin assignments
- PID gains (`YAW_KP`, `YAW_KI`, `YAW_KD`)
- Distance thresholds (`OBSTACLE_THRESHOLD_CM`, `UPBAR_THRESHOLD_CM`)
- Motion settings (`THRUST_CRUISE_DUTY`, `TURN_ANGLE_DEG`, timeouts)

---

## Media
| Drawing | Model | Competition setup |
| :--: | :--: | :--: |
| ![Drawing](drawing.png) | ![Model](model.png) | ![Competition setup](final_comp.png) |

---

## Legacy firmware
The original single-file implementation is preserved at:
- `legacy/FinalHovercraftCode_290_TEAM3_FALL_2025.c`
