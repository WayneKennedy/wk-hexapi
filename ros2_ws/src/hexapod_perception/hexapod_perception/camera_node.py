#!/usr/bin/env python3
"""
Camera Node for Hexapod Robot
Publishes camera frames to ROS 2 topics

Supports picamera2 (Pi Camera) and OpenCV (USB/V4L2) backends
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from threading import Condition
import numpy as np
import io

# Camera backend imports with fallback
PICAMERA_AVAILABLE = False
OPENCV_AVAILABLE = False

try:
    from picamera2 import Picamera2
    from libcamera import Transform
    PICAMERA_AVAILABLE = True
except (ImportError, ModuleNotFoundError) as e:
    # picamera2 may fail due to missing preview dependencies (pykms)
    # This is expected in headless Docker environments
    pass

try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    pass


class StreamingOutput(io.BufferedIOBase):
    """Thread-safe frame buffer for picamera2 streaming"""

    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf: bytes) -> int:
        with self.condition:
            self.frame = buf
            self.condition.notify_all()
        return len(buf)


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare parameters
        self.declare_parameter('camera.width', 640)
        self.declare_parameter('camera.height', 480)
        self.declare_parameter('camera.fps', 30.0)
        self.declare_parameter('camera.horizontal_flip', False)
        self.declare_parameter('camera.vertical_flip', False)
        self.declare_parameter('camera.backend', 'picamera2')
        self.declare_parameter('camera.device_id', 0)

        # Get parameters
        self.width = self.get_parameter('camera.width').value
        self.height = self.get_parameter('camera.height').value
        self.fps = self.get_parameter('camera.fps').value
        self.hflip = self.get_parameter('camera.horizontal_flip').value
        self.vflip = self.get_parameter('camera.vertical_flip').value
        self.backend = self.get_parameter('camera.backend').value
        self.device_id = self.get_parameter('camera.device_id').value

        self.camera = None
        self.streaming_output = None
        self.cap = None  # OpenCV capture

        # Initialize camera based on backend preference
        if self.backend == 'picamera2' and PICAMERA_AVAILABLE:
            self._init_picamera()
        elif OPENCV_AVAILABLE:
            self._init_opencv()
        else:
            self.get_logger().warn(
                'No camera backend available, running in simulation mode'
            )

        # Create publisher
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)

        # Create timer for publishing frames
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)

        self.get_logger().info(
            f'Camera node started: {self.width}x{self.height} @ {self.fps} fps'
        )

    def _init_picamera(self):
        """Initialize Pi Camera using picamera2"""
        try:
            self.camera = Picamera2()

            transform = Transform(
                hflip=1 if self.hflip else 0,
                vflip=1 if self.vflip else 0
            )

            # Configure for video capture with RGB output
            config = self.camera.create_video_configuration(
                main={'size': (self.width, self.height), 'format': 'RGB888'},
                transform=transform
            )
            self.camera.configure(config)
            self.camera.start()

            self.get_logger().info(
                f'Pi Camera initialized ({self.width}x{self.height})'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Pi Camera: {e}')
            self.camera = None

            # Try OpenCV as fallback
            if OPENCV_AVAILABLE:
                self.get_logger().info('Falling back to OpenCV backend')
                self._init_opencv()

    def _init_opencv(self):
        """Initialize camera using OpenCV"""
        try:
            self.cap = cv2.VideoCapture(self.device_id)

            if not self.cap.isOpened():
                raise RuntimeError(f'Cannot open camera device {self.device_id}')

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

            # Test read to verify camera works
            ret, _ = self.cap.read()
            if not ret:
                raise RuntimeError('Camera opened but cannot read frames (Pi Camera needs libcamera)')

            # Read actual settings (may differ from requested)
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            self.get_logger().info(
                f'OpenCV camera initialized ({actual_width}x{actual_height})'
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to initialize OpenCV camera: {e}')
            self.get_logger().info('Running in simulation mode (test pattern)')
            if self.cap:
                self.cap.release()
            self.cap = None

    def publish_frame(self):
        """Capture and publish a camera frame"""
        frame = None

        if self.camera is not None:
            # picamera2 backend
            try:
                frame = self.camera.capture_array()
            except Exception as e:
                self.get_logger().warn(f'Failed to capture frame (picamera2): {e}')
                return

        elif self.cap is not None:
            # OpenCV backend
            try:
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn('Failed to capture frame (OpenCV)')
                    return
                # Convert BGR to RGB
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # Apply flip if needed
                if self.hflip:
                    frame = cv2.flip(frame, 1)
                if self.vflip:
                    frame = cv2.flip(frame, 0)

            except Exception as e:
                self.get_logger().warn(f'Failed to capture frame (OpenCV): {e}')
                return

        else:
            # Simulation mode - generate test pattern
            frame = self._generate_test_pattern()

        # Create and publish ROS Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()

        self.image_pub.publish(msg)

    def _generate_test_pattern(self):
        """Generate a test pattern for simulation mode"""
        # Create gradient test pattern
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Horizontal gradient (red channel)
        frame[:, :, 0] = np.tile(
            np.linspace(0, 255, self.width, dtype=np.uint8),
            (self.height, 1)
        )

        # Vertical gradient (green channel)
        frame[:, :, 1] = np.tile(
            np.linspace(0, 255, self.height, dtype=np.uint8).reshape(-1, 1),
            (1, self.width)
        )

        # Add timestamp text position indicator (blue box)
        t = int(self.get_clock().now().nanoseconds / 1e8) % self.width
        frame[10:30, t:t+20, 2] = 255

        return frame

    def destroy_node(self):
        """Clean up camera resources"""
        if self.camera is not None:
            try:
                self.camera.stop()
                self.camera.close()
            except Exception:
                pass

        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
