# wk-hexapi

ROS 2 based autonomous hexapod robot, built on Freenove Big Hexapod hardware.

## Hardware

- Raspberry Pi 5 (8GB)
- 20 servos (18 leg + 2 head pan/tilt) via PCA9685
- OV5647 camera (on pan/tilt head)
- Ultrasonic sensor (on pan/tilt head)
- MPU6050 IMU
- ADS7830 ADC for dual battery monitoring
- WS2812 LEDs
- Buzzer

## Goals

- ROS 2 Jazzy on Ubuntu Server 24.04
- Autonomous mapping and navigation (SLAM + Nav2)
- Wander mode with return-to-home capability

## Project Structure

```
wk-hexapi/
├── reference/          # Original Freenove code for reference
├── config/             # Calibration data and configs
├── ros2_ws/            # ROS 2 workspace (to be created on Pi)
└── docs/               # Documentation
```

## License

Apache 2.0
