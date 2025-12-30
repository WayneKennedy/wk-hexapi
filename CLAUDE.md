# Project Context

## Overview
ROS 2 based autonomous hexapod robot project, built on Freenove Big Hexapod hardware (FNK0052).

## Target Platform
- **Hardware**: Raspberry Pi 5 (8GB)
- **OS**: Ubuntu Server 24.04
- **ROS**: ROS 2 Jazzy

## Hardware Components
- 20 servos (18 leg + 2 head pan/tilt) via PCA9685
- OV5647 camera on pan/tilt head
- Ultrasonic sensor on pan/tilt head
- MPU6050 IMU
- ADS7830 ADC for dual battery monitoring
- WS2812 LEDs
- Buzzer

## Project Goals
- Autonomous mapping and navigation (SLAM + Nav2)
- Wander mode with return-to-home capability

## Development Environment
- Docker-based development (recommended)
- ROS 2 workspace at `ros2_ws/`
- Reference Freenove code in `reference/`

## Key Directories
- `ros2_ws/src/hexapod_hardware/` - Hardware interface nodes
- `ros2_ws/src/hexapod_bringup/` - Launch files and config
- `config/` - Hardware calibration
- `scripts/` - Setup scripts
