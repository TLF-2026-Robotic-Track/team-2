#!/usr/bin/python3
"""Track a target color and drive toward it with two PID loops.

The camera publishes x in normalized coordinates, where x = 0.5 is the center of
image. The left motor uses the direct PID output and the right motor uses the
inverse PID output. If no target is found, both motors stop.
"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Header, String

from duckietown_msgs.msg import WheelsCmdStamped

# ---- PID and motion tuning ---------------------------------------------------
P = 1.6
I = 0.05
D = 0.25
SETPOINT_X = 0.5
CENTER_TOLERANCE = 0.05
BASE_SPEED = 0.50
MAX_MOTOR_SPEED = 0.8
SEARCH_TURN_SPEED = 0.60
TARGET_MIN_AREA = 200
TARGET_MAX_AREA_PERCENT = 20.0
TARGET_COLOR = os.environ.get('TARGET_COLOR', 'green').lower()
RESTART_COLOR = 'blue'
RESTART_MIN_AREA = 200
RESTART_CENTER_TOLERANCE = 0.25
BLUE_START_DELAY_S = 2.0
# ----------------------------------------------------------------------------

COLOR_RANGES = {
    'red': [
        ((0, 80, 80), (10, 255, 255)),
        ((160, 80, 80), (180, 255, 255)),
    ],
    'green': [
        ((35, 50, 50), (90, 255, 255)),
    ],
    'blue': [
        ((95, 80, 50), (135, 255, 255)),
    ],
    'yellow': [
        ((20, 80, 80), (40, 255, 255)),
    ],
    'orange': [
        ((5, 80, 80), (22, 255, 255)),
    ],
}

if TARGET_COLOR not in COLOR_RANGES:
    raise SystemExit(
        f'Unsupported TARGET_COLOR={TARGET_COLOR!r}. Use one of: '
        + ', '.join(sorted(COLOR_RANGES.keys()))
    )


class ColorDetector(Node):
    def __init__(self, vehicle_name):
        super().__init__('color_detector')

        self.blob_pub = self.create_publisher(
            Float32MultiArray,
            f'/{vehicle_name}/color_blob',
            10,
        )
        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped,
            f'/{vehicle_name}/wheels_cmd',
            10,
        )
        self.led_pub = self.create_publisher(
            String,
            f'/{vehicle_name}/LED_commands',
            10,
        )
        self.largest_color_pub = self.create_publisher(
            String,
            f'/{vehicle_name}/largest_color',
            10,
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            f'/{vehicle_name}/image/compressed',
            self.on_image,
            10,
        )
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.waiting_for_blue = False
        self.blue_start_time = None
        self.blue_event_sent = False
        self.blue_start_timer = self.create_timer(0.05, self.check_blue_start)

        self.get_logger().info(
            f'Detecting {TARGET_COLOR} blobs on {vehicle_name}; '
            f'P={P}, I={I}, D={D}, base speed={BASE_SPEED}, '
            f'blue start delay={BLUE_START_DELAY_S}s'
        )

    def _build_mask(self, hsv_image, color=TARGET_COLOR):
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for lower, upper in COLOR_RANGES[color]:
            lower_hsv = np.array(lower, dtype=np.uint8)
            upper_hsv = np.array(upper, dtype=np.uint8)
            mask |= cv2.inRange(hsv_image, lower_hsv, upper_hsv)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def find_largest_color(self, frame, color):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._build_mask(hsv, color)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < TARGET_MIN_AREA:
            return None
        return largest

    def find_largest_detected_color(self, frame):
        largest_color = None
        largest_contour = None
        largest_area = 0.0

        for color in COLOR_RANGES:
            contour = self.find_largest_color(frame, color)
            if contour is None:
                continue
            area = cv2.contourArea(contour)
            if area > largest_area:
                largest_color = color
                largest_contour = contour
                largest_area = area

        return largest_color, largest_contour

    def is_restart_key(self, frame):
        blue_contour = self.find_largest_color(frame, RESTART_COLOR)
        if blue_contour is None:
            return False

        blue_area = cv2.contourArea(blue_contour)
        if blue_area < RESTART_MIN_AREA:
            return False

        blue_x, _, blue_width, _ = cv2.boundingRect(blue_contour)
        blue_center_x = blue_x + blue_width / 2.0
        center_x = frame.shape[1] / 2.0
        center_error = abs(blue_center_x - center_x) / max(frame.shape[1], 1)
        if center_error > RESTART_CENTER_TOLERANCE:
            return False

        _, largest_contour = self.find_largest_detected_color(frame)
        return (
            largest_contour is not None
            and blue_area >= cv2.contourArea(largest_contour)
        )

    def publish_blob(self, x_center_norm, area_ratio, detected):
        msg = Float32MultiArray()
        msg.data = [float(x_center_norm), float(area_ratio), float(detected)]
        self.blob_pub.publish(msg)

    def publish_largest_color(self, color):
        msg = String()
        msg.data = color if color is not None else 'nan'
        self.largest_color_pub.publish(msg)

    def publish_led_state(self, state):
        msg = String()
        msg.data = state
        self.led_pub.publish(msg)

    def publish_wheels(self, left_speed, right_speed):
        msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'color_pid'
        msg.header = header
        msg.vel_left = float(left_speed)
        msg.vel_right = float(right_speed)
        self.wheels_pub.publish(msg)

    def start_search(self, reason):
        self.blue_start_time = None
        self.blue_event_sent = False
        self.publish_led_state('nan')
        self.publish_wheels(SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED)
        self.get_logger().info(f'Starting clockwise bottle search: {reason}')

    def check_blue_start(self):
        if self.blue_start_time is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.blue_start_time >= BLUE_START_DELAY_S:
                self.waiting_for_blue = False
                self.start_search('2 seconds after blue key')
            else:
                if not self.blue_event_sent:
                    self.publish_led_state('blue')
                    self.blue_event_sent = True
                self.publish_wheels(0.0, 0.0)

    def pid(self, x_center_norm):
        # Positive error means the target is to the right.
        error = x_center_norm - SETPOINT_X
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.last_time is None:
            dt = 0.02
        else:
            dt = max(now - self.last_time, 1e-3)

        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = P * error + I * self.integral + D * derivative

        self.last_error = error
        self.last_time = now
        return output

    def no_color(self):
        if self.waiting_for_blue or self.blue_start_time is not None:
            self.publish_led_state('finished')
            self.publish_wheels(0.0, 0.0)
            return

        self.publish_blob(0.0, 0.0, 0.0)
        self.publish_led_state('nan')
        # Search only clockwise to avoid the rapid left/right oscillation.
        self.publish_wheels(SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED)
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def bottle_found(self):
        self.waiting_for_blue = True
        self.publish_led_state('finished')
        self.publish_wheels(0.0, 0.0)

    def move_toward_bottle(self, x_center_norm):
        if self.waiting_for_blue:
            self.publish_led_state('finished')
            self.publish_wheels(0.0, 0.0)
            return

        error = x_center_norm - SETPOINT_X
        turn_correction = 0.0
        if abs(error) > CENTER_TOLERANCE:
            turn_correction = self.pid(x_center_norm)

        # Drive at maximum speed and reduce the wheel on the side of the turn.
        left_speed = MAX_MOTOR_SPEED - max(0.0, -turn_correction)
        right_speed = MAX_MOTOR_SPEED - max(0.0, turn_correction)

        left_speed = max(0.0, min(MAX_MOTOR_SPEED, left_speed))
        right_speed = max(0.0, min(MAX_MOTOR_SPEED, right_speed))

        self.publish_led_state('bottle')
        self.publish_wheels(left_speed, right_speed)

    def on_image(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as exc:  # pragma: no cover - runtime safety
            self.get_logger().warn(f'Could not decode image: {exc}')
            self.no_color()
            return

        if frame is None or frame.size == 0:
            self.no_color()
            return

        largest_color, largest_contour = self.find_largest_detected_color(frame)
        self.publish_largest_color(largest_color)

        if self.waiting_for_blue:
            if not self.is_restart_key(frame):
                self.publish_led_state('finished')
                self.publish_wheels(0.0, 0.0)
                return

            # Blue is the ignition key. Keep stopped for the configured delay.
            self.waiting_for_blue = False
            self.blue_start_time = self.get_clock().now().nanoseconds * 1e-9
            self.blue_event_sent = False
            self.get_logger().info('Blue ignition detected; waiting 2 seconds')
            self.publish_led_state('blue')
            self.publish_wheels(0.0, 0.0)
            return

        if self.blue_start_time is not None:
            self.publish_led_state('blue')
            self.publish_wheels(0.0, 0.0)
            return

        largest = self.find_largest_color(frame, TARGET_COLOR)
        if largest is None:
            self.no_color()
            return

        x, y, w, h = cv2.boundingRect(largest)
        x_center = x + w / 2.0
        x_center_norm = x_center / max(frame.shape[1], 1)
        area_px = cv2.contourArea(largest)
        image_area = max(frame.shape[0] * frame.shape[1], 1)
        area_ratio = area_px / image_area

        self.publish_blob(x_center_norm, area_ratio, 1.0)

        if area_ratio * 100.0 >= TARGET_MAX_AREA_PERCENT:
            self.bottle_found()
            return

        self.move_toward_bottle(x_center_norm)


def main():
    vehicle_name = os.environ.get('VEHICLE_NAME')
    if not vehicle_name:
        raise SystemExit('VEHICLE_NAME is not set. Run: export VEHICLE_NAME=duckie03')

    rclpy.init()
    node = ColorDetector(vehicle_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_wheels(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
