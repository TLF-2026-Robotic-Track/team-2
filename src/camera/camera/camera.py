#!/usr/bin/python3
"""Save pictures from the Duckiebot camera to disk.

Subscribes to /<VEHICLE_NAME>/image/compressed. The camera sends about 30
frames per second, which is far too many to keep, so only every EVERY_NTH
frame is written to a file.

The images land in /workspace/images inside the container, which is the
images/ folder of this repository on the host.
"""
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# ---- change these ----------------------------------------------------------
EVERY_NTH = 30                  # save 1 frame out of this many (30 ~= 1 per second)
OUTPUT_DIR = os.environ.get('IMAGE_DIR', '/workspace/images')
# ----------------------------------------------------------------------------


class Camera(Node):
    def __init__(self, vehicle_name):
        super().__init__('camera')
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        self.frames = 0          # how many frames arrived
        self.saved = 0           # how many we wrote to disk

        self.create_subscription(
            CompressedImage,
            f'/{vehicle_name}/image/compressed',
            self.on_image,
            10)

        self.get_logger().info(
            f'Saving every {EVERY_NTH}th frame of {vehicle_name} to {OUTPUT_DIR}')

    def on_image(self, msg):
        self.frames += 1
        if self.frames % EVERY_NTH != 0:
            return

        path = os.path.join(OUTPUT_DIR, f'{self.saved:04d}.jpg')
        with open(path, 'wb') as f:
            f.write(msg.data)

        self.saved += 1
        self.get_logger().info(f'Saved {path} ({len(msg.data)} bytes)')


def main():
    vehicle_name = os.environ.get('VEHICLE_NAME')
    if not vehicle_name:
        raise SystemExit('VEHICLE_NAME is not set. Run: export VEHICLE_NAME=duckie03')

    rclpy.init()
    node = Camera(vehicle_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
