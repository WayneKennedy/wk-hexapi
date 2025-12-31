#!/usr/bin/env python3
"""
Test script: Initialize, stand, walk forward 150mm, reverse 150mm
Direct hardware test - no ROS required.
"""

import copy
import math
import numpy as np
import time
import smbus


class PCA9685:
    """PCA9685 PWM driver"""
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus, address=0x40):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)

    def set_pwm_freq(self, freq):
        prescale = int(25000000.0 / (4096.0 * freq) - 1 + 0.5)
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        self.bus.write_byte_data(self.address, self.MODE1, (old_mode & 0x7F) | 0x10)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0x80)

    def set_pwm(self, channel, on, off):
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 1, on >> 8)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 3, off >> 8)

    def set_pwm_off(self, channel):
        self.set_pwm(channel, 4096, 4096)


class HexapodTest:
    """Direct hardware test for hexapod controller"""

    LEG_CHANNELS = [
        [15, 14, 13],  # Leg 0 (RF)
        [12, 11, 10],  # Leg 1 (RM)
        [9, 8, 31],    # Leg 2 (RR)
        [22, 23, 27],  # Leg 3 (LR)
        [19, 20, 21],  # Leg 4 (LM)
        [16, 17, 18],  # Leg 5 (LF)
    ]

    L1, L2, L3 = 33.0, 90.0, 110.0  # Link lengths

    def __init__(self):
        print("Initializing PCA9685 drivers...")
        self.pwm_41 = PCA9685(1, 0x41)
        self.pwm_41.set_pwm_freq(50)
        self.pwm_40 = PCA9685(1, 0x40)
        self.pwm_40.set_pwm_freq(50)
        time.sleep(0.1)

        # Body geometry
        self.leg_angles = [54.0, 0.0, -54.0, -126.0, 180.0, 126.0]
        self.leg_offsets = [94.0, 85.0, 94.0, 94.0, 85.0, 94.0]
        self.height_offset = 14.0
        self.default_height = 30.0

        # State
        self.body_position = np.array([0.0, 0.0, 0.0])
        self.body_orientation = np.array([0.0, 0.0, 0.0])
        self.foot_positions = np.array([
            [137.1, 189.4, 0.0],
            [225.0, 0.0, 0.0],
            [137.1, -189.4, 0.0],
            [-137.1, -189.4, 0.0],
            [-225.0, 0.0, 0.0],
            [-137.1, 189.4, 0.0],
        ])
        self.leg_positions = [[140.0, 0.0, 0.0] for _ in range(6)]
        self.current_angles = [[90.0, 90.0, 90.0] for _ in range(6)]

        # Load calibration
        self.calibration_positions = self._load_calibration()
        self.calibration_angles = [[0.0, 0.0, 0.0] for _ in range(6)]
        self._calculate_calibration()

        print("Ready")

    def _load_calibration(self):
        paths = [
            '/home/wkenn/Code/wk-hexapod/ros2_ws/src/hexapod_hardware/config/servo_calibration.txt',
            '/home/wkenn/Code/fn-hexapod/Code/Server/point.txt',
        ]
        for path in paths:
            try:
                with open(path, 'r') as f:
                    data = [list(map(int, line.strip().split('\t')))
                            for line in f if line.strip()]
                    print(f"Loaded calibration from {path}")
                    return data
            except:
                pass
        print("Using default calibration")
        return [[140, 0, 0] for _ in range(6)]

    def _calculate_calibration(self):
        for i in range(6):
            cal = self._coordinate_to_angle(
                -self.calibration_positions[i][2],
                self.calibration_positions[i][0],
                self.calibration_positions[i][1])
            default = self._coordinate_to_angle(0, 140, 0)
            self.calibration_angles[i] = [cal[j] - default[j] for j in range(3)]

    def _clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    def _coordinate_to_angle(self, x, y, z):
        """IK: leg endpoint to servo angles"""
        a = math.pi / 2 - math.atan2(z, y)
        x_4 = self.L1 * math.sin(a)
        x_5 = self.L1 * math.cos(a)
        l23 = math.sqrt((z - x_5)**2 + (y - x_4)**2 + x**2)

        w = self._clamp(x / l23, -1, 1)
        v = self._clamp((self.L2**2 + l23**2 - self.L3**2) / (2 * self.L2 * l23), -1, 1)
        u = self._clamp((self.L2**2 + self.L3**2 - l23**2) / (2 * self.L3 * self.L2), -1, 1)

        b = math.asin(round(w, 2)) - math.acos(round(v, 2))
        c = math.pi - math.acos(round(u, 2))

        return [round(math.degrees(a)), round(math.degrees(b)), round(math.degrees(c))]

    def _world_to_leg(self, foot_pos, leg_idx):
        """World frame to leg-local frame"""
        angle_rad = math.radians(self.leg_angles[leg_idx])
        offset = self.leg_offsets[leg_idx]

        x = (foot_pos[0] * math.cos(angle_rad) +
             foot_pos[1] * math.sin(angle_rad) - offset)
        y = (-foot_pos[0] * math.sin(angle_rad) +
             foot_pos[1] * math.cos(angle_rad))
        z = foot_pos[2] - self.height_offset

        return [x, y, z]

    def _calculate_rotation_matrix(self, roll, pitch, yaw):
        r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
        Rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])
        Ry = np.array([[math.cos(r), 0, -math.sin(r)], [0, 1, 0], [math.sin(r), 0, math.cos(r)]])
        Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
        return Rx @ Ry @ Rz

    def _apply_body_transform(self):
        R = self._calculate_rotation_matrix(*self.body_orientation)
        for i in range(6):
            foot = np.array(self.foot_positions[i])
            transformed = R @ foot + self.body_position
            self.leg_positions[i] = self._world_to_leg(transformed, i)

    def _set_servo_angle(self, channel, angle):
        angle = self._clamp(angle, 0, 180)
        duty = (angle / 180.0) * (2500 - 500) + 500
        pwm_val = int(duty / 20000.0 * 4095)

        if channel < 16:
            self.pwm_41.set_pwm(channel, 0, pwm_val)
        elif channel < 32:
            self.pwm_40.set_pwm(channel - 16, 0, pwm_val)

    def _update_servos(self):
        """Calculate and send all servo angles"""
        self._apply_body_transform()

        for i in range(6):
            raw = self._coordinate_to_angle(
                -self.leg_positions[i][2],
                self.leg_positions[i][0],
                self.leg_positions[i][1])

            if i < 3:  # Right side
                self.current_angles[i][0] = self._clamp(raw[0] + self.calibration_angles[i][0], 0, 180)
                self.current_angles[i][1] = self._clamp(90 - (raw[1] + self.calibration_angles[i][1]), 0, 180)
                self.current_angles[i][2] = self._clamp(raw[2] + self.calibration_angles[i][2], 0, 180)
            else:  # Left side
                self.current_angles[i][0] = self._clamp(raw[0] + self.calibration_angles[i][0], 0, 180)
                self.current_angles[i][1] = self._clamp(90 + raw[1] + self.calibration_angles[i][1], 0, 180)
                self.current_angles[i][2] = self._clamp(180 - (raw[2] + self.calibration_angles[i][2]), 0, 180)

        for leg_idx in range(6):
            channels = self.LEG_CHANNELS[leg_idx]
            for j in range(3):
                self._set_servo_angle(channels[j], self.current_angles[leg_idx][j])

    def _relax_servos(self):
        for i in range(16):
            self.pwm_40.set_pwm_off(i)
            self.pwm_41.set_pwm_off(i)
        print("Servos relaxed")

    # ===== Commands =====

    def home(self):
        """Move to calibrated home position"""
        print("HOME...")
        self.body_position = np.array([0.0, 0.0, 0.0])
        self.body_orientation = np.array([0.0, 0.0, 0.0])
        self.foot_positions = np.array([
            [137.1, 189.4, 0.0],
            [225.0, 0.0, 0.0],
            [137.1, -189.4, 0.0],
            [-137.1, -189.4, 0.0],
            [-225.0, 0.0, 0.0],
            [-137.1, 189.4, 0.0],
        ])
        self._update_servos()
        print("HOME done")

    def stand(self, height=None, duration=1.0):
        """Raise body to standing height using body position control"""
        if height is None:
            height = self.default_height

        print(f"STAND (raising body {height}mm)...")
        steps = 50
        start_z = self.body_position[2]
        end_z = -height

        for step in range(steps + 1):
            t = step / steps
            t = t * t * (3 - 2 * t)  # Ease in-out
            self.body_position[2] = start_z + t * (end_z - start_z)
            self._update_servos()
            time.sleep(duration / steps)

        print("STAND done")

    def walk(self, distance_mm, step_size=25, step_height=40.0, cycle_time=0.8):
        """Walk using tripod gait"""
        direction = "FORWARD" if distance_mm > 0 else "BACKWARD"
        print(f"WALK {direction} {abs(distance_mm)}mm...")

        cycles = int(abs(distance_mm) / step_size)
        x_per_step = step_size if distance_mm > 0 else -step_size

        for cycle in range(cycles):
            print(f"  Step {cycle+1}/{cycles}")
            self._tripod_gait_cycle(x_per_step, 0, 0, step_height, cycle_time)

        # Return to neutral
        self._reset_stance()
        print(f"WALK {direction} done")

    def _reset_stance(self):
        """Reset to neutral standing stance"""
        self.foot_positions = np.array([
            [137.1, 189.4, self.body_position[2]],
            [225.0, 0.0, self.body_position[2]],
            [137.1, -189.4, self.body_position[2]],
            [-137.1, -189.4, self.body_position[2]],
            [-225.0, 0.0, self.body_position[2]],
            [-137.1, 189.4, self.body_position[2]],
        ])
        self._update_servos()

    def _tripod_gait_cycle(self, x, y, turn, step_height, cycle_time):
        """One full tripod gait cycle"""
        angle_rad = math.radians(turn)
        base_points = copy.deepcopy(self.foot_positions)

        # Movement delta for each leg
        xy_delta = []
        for i in range(6):
            rot_x = (base_points[i][0] * math.cos(angle_rad) +
                     base_points[i][1] * math.sin(angle_rad) - base_points[i][0])
            rot_y = (-base_points[i][0] * math.sin(angle_rad) +
                     base_points[i][1] * math.cos(angle_rad) - base_points[i][1])
            xy_delta.append([rot_x + x, rot_y + y])

        frames = 64
        z_delta = step_height / frames
        delay = cycle_time / frames

        for j in range(frames):
            for i in range(3):
                even, odd = 2 * i, 2 * i + 1

                if j < frames // 8:
                    base_points[even][0] -= 4 * xy_delta[even][0] / frames
                    base_points[even][1] -= 4 * xy_delta[even][1] / frames
                    base_points[odd][0] += 8 * xy_delta[odd][0] / frames
                    base_points[odd][1] += 8 * xy_delta[odd][1] / frames
                    base_points[odd][2] = self.body_position[2] + step_height
                elif j < frames // 4:
                    base_points[even][0] -= 4 * xy_delta[even][0] / frames
                    base_points[even][1] -= 4 * xy_delta[even][1] / frames
                    base_points[odd][2] -= z_delta * 8
                elif j < 3 * frames // 8:
                    base_points[even][2] += z_delta * 8
                    base_points[odd][0] -= 4 * xy_delta[odd][0] / frames
                    base_points[odd][1] -= 4 * xy_delta[odd][1] / frames
                elif j < 5 * frames // 8:
                    base_points[even][0] += 8 * xy_delta[even][0] / frames
                    base_points[even][1] += 8 * xy_delta[even][1] / frames
                    base_points[odd][0] -= 4 * xy_delta[odd][0] / frames
                    base_points[odd][1] -= 4 * xy_delta[odd][1] / frames
                elif j < 3 * frames // 4:
                    base_points[even][2] -= z_delta * 8
                    base_points[odd][0] -= 4 * xy_delta[odd][0] / frames
                    base_points[odd][1] -= 4 * xy_delta[odd][1] / frames
                elif j < 7 * frames // 8:
                    base_points[even][0] -= 4 * xy_delta[even][0] / frames
                    base_points[even][1] -= 4 * xy_delta[even][1] / frames
                    base_points[odd][2] += z_delta * 8
                else:
                    base_points[even][0] -= 4 * xy_delta[even][0] / frames
                    base_points[even][1] -= 4 * xy_delta[even][1] / frames
                    base_points[odd][0] += 8 * xy_delta[odd][0] / frames
                    base_points[odd][1] += 8 * xy_delta[odd][1] / frames

            self.foot_positions = np.array(base_points)
            self._update_servos()
            time.sleep(delay)


def main():
    print("=" * 50)
    print("Hexapod Walk Test")
    print("HOME -> STAND -> FORWARD 150mm -> BACKWARD 150mm")
    print("=" * 50)

    try:
        robot = HexapodTest()

        input("\nPress Enter to HOME...")
        robot.home()
        time.sleep(0.5)

        input("\nPress Enter to STAND...")
        robot.stand()
        time.sleep(0.5)

        input("\nPress Enter to WALK FORWARD 150mm...")
        robot.walk(150)
        time.sleep(0.5)

        input("\nPress Enter to WALK BACKWARD 150mm...")
        robot.walk(-150)
        time.sleep(0.5)

        input("\nPress Enter to RELAX...")
        robot._relax_servos()

        print("\n=== TEST COMPLETE ===")

    except KeyboardInterrupt:
        print("\nAborted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
