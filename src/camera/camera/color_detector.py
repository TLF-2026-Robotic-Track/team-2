#!/usr/bin/python3
"""Detect a target color in the camera stream and publish its x-position and area.

Subscribes to /<VEHICLE_NAME>/image/compressed and publishes a Float32MultiArray on
/<VEHICLE_NAME>/color_blob. The array contains:

    [x_center_normalized, area_ratio, detected]

where x_center_normalized is in the range [0, 1], area_ratio is the contour area
as a fraction of the full image area, and detected is 1.0 when a target is found.
"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

TARGET_COLOR = os.environ.get('TARGET_COLOR', 'green').lower()
TARGET_MIN_AREA = max(1, int(os.environ.get('TARGET_MIN_AREA', '200')))

COLOR_RANGES = {
    'red': [
        ((0, 80, 80), (10, 255, 255)),
        ((160, 80, 80), (180, 255, 255)),
    ],
    'green': [
        ((35, 50, 50), (90, 255, 255)),
    ],
    'blue': [
        ((100, 80, 80), (140, 255, 255)),
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

        self.publisher = self.create_publisher(
            Float32MultiArray,
            f'/{vehicle_name}/color_blob',
            10,
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            f'/{vehicle_name}/image/compressed',
            self.on_image,
            10,
        )

        self.get_logger().info(
            f'Detecting {TARGET_COLOR} blobs on {vehicle_name}; '
            f'min area={TARGET_MIN_AREA}; publishing to /{vehicle_name}/color_blob'
        )

    def _build_mask(self, hsv_image):
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for lower, upper in COLOR_RANGES[TARGET_COLOR]:
            lower_bgr = np.array(lower, dtype=np.uint8)
            upper_bgr = np.array(upper, dtype=np.uint8)
            mask |= cv2.inRange(hsv_image, lower_bgr, upper_bgr)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def publish_result(self, x_center_norm, area_ratio, detected):
        msg = Float32MultiArray()
        msg.data = [float(x_center_norm), float(area_ratio), float(detected)]
        self.publisher.publish(msg)

    def on_image(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as exc:  # pragma: no cover - runtime safety
            self.get_logger().warn(f'Could not decode image: {exc}')
            return

        if frame is None or frame.size == 0:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._build_mask(hsv)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.publish_result(0.0, 0.0, 0.0)
            return

        largest = max(contours, key=cv2.contourArea)
        area_px = cv2.contourArea(largest)
        image_area = frame.shape[0] * frame.shape[1]

        #if area_px < TARGET_MIN_AREA:
        #    self.publish_result(0.0, 0.0, 0.0)
        #    return

        x, y, w, h = cv2.boundingRect(largest)
        x_center = x + w / 2.0
        x_center_norm = x_center / max(frame.shape[1], 1)
        area_ratio = area_px / max(image_area, 1)

        self.publish_result(x_center_norm, area_ratio*100, 1.0)


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
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
