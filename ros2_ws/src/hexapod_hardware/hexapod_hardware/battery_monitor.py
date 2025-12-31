#!/usr/bin/env python3
"""
Battery Monitor Node for Hexapod Robot
Monitors dual battery voltages via ADS7830 ADC

Based on working implementation in ../fn-hexapod/Code/Server/adc.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Hardware imports
try:
    import smbus
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class ADS7830:
    """ADS7830 ADC driver - adapted from fn-hexapod/Code/Server/adc.py"""

    ADS7830_COMMAND = 0x84
    ADC_VOLTAGE_COEFFICIENT = 3  # Based on PCB voltage divider

    def __init__(self, bus, address=0x48):
        self.bus = smbus.SMBus(bus)
        self.address = address

    def _read_stable_byte(self):
        """Read a stable byte from the ADC (reads twice until values match)"""
        while True:
            value1 = self.bus.read_byte(self.address)
            value2 = self.bus.read_byte(self.address)
            if value1 == value2:
                return value1

    def read_channel(self, channel):
        """Read single-ended ADC channel (0-7)"""
        # Command byte formula from fn-hexapod adc.py
        command = self.ADS7830_COMMAND | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)
        self.bus.write_byte(self.address, command)
        return self._read_stable_byte()

    def read_voltage(self, channel):
        """Read voltage with scaling for voltage divider"""
        # Formula from fn-hexapod: value / 255.0 * 5 * coefficient
        raw = self.read_channel(channel)
        voltage = raw / 255.0 * 5 * self.ADC_VOLTAGE_COEFFICIENT
        return voltage


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # Declare parameters
        self.declare_parameter('i2c.bus', 1)
        self.declare_parameter('i2c.ads7830_addr', 0x48)
        self.declare_parameter('battery.publish_rate', 1.0)
        self.declare_parameter('battery.low_voltage_warn', 6.5)
        # Physical switches: LOAD (servos) and CTRL (Pi)
        self.declare_parameter('battery.load_channel', 0)  # LOAD battery (servos)
        self.declare_parameter('battery.ctrl_channel', 4)  # CTRL battery (Pi)

        # Get parameters
        bus = self.get_parameter('i2c.bus').value
        addr = self.get_parameter('i2c.ads7830_addr').value
        publish_rate = self.get_parameter('battery.publish_rate').value
        self.low_voltage = self.get_parameter('battery.low_voltage_warn').value
        self.load_channel = self.get_parameter('battery.load_channel').value
        self.ctrl_channel = self.get_parameter('battery.ctrl_channel').value

        # Initialize ADC
        self.adc = None
        if HARDWARE_AVAILABLE:
            try:
                self.adc = ADS7830(bus, addr)
                self.get_logger().info(f'ADS7830 initialized at 0x{addr:02x}')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize ADS7830: {e}')
        else:
            self.get_logger().warn('Hardware not available, running in simulation mode')

        # Create publishers
        self.voltage_pub = self.create_publisher(
            Float32MultiArray, 'battery/voltages', 10
        )
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )

        # Create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_battery_status)

        self.get_logger().info(f'Battery monitor started at {publish_rate} Hz')

    def publish_battery_status(self):
        # Read batteries: LOAD (servos) and CTRL (Pi)
        if self.adc:
            try:
                load_voltage = self.adc.read_voltage(self.load_channel)
                ctrl_voltage = self.adc.read_voltage(self.ctrl_channel)
            except Exception as e:
                self.get_logger().warn(f'Failed to read ADC: {e}')
                return
        else:
            # Simulation mode
            load_voltage = 7.4
            ctrl_voltage = 7.4

        # Publish voltages [LOAD, CTRL]
        msg = Float32MultiArray()
        msg.data = [load_voltage, ctrl_voltage]
        self.voltage_pub.publish(msg)

        # Publish diagnostics with physical switch names
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        batteries = [
            ('LOAD', 'battery_load', load_voltage, 'Servos'),
            ('CTRL', 'battery_ctrl', ctrl_voltage, 'Pi'),
        ]

        for name, hw_id, voltage, desc in batteries:
            status = DiagnosticStatus()
            status.name = f'Battery {name}'
            status.hardware_id = hw_id

            if voltage < self.low_voltage:
                status.level = DiagnosticStatus.WARN
                status.message = f'Low voltage ({desc})'
            else:
                status.level = DiagnosticStatus.OK
                status.message = f'OK ({desc})'

            status.values.append(KeyValue(key='voltage', value=f'{voltage:.2f}V'))
            status.values.append(KeyValue(key='description', value=desc))
            diag.status.append(status)

        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
