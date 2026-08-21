#!/usr/bin/python3
"""Blink the Duckiebot LEDs: red -> blue, in a fast police strobe style.

Publishes to /<VEHICLE_NAME>/led_pattern.
"""
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA

from duckietown_msgs.msg import LEDPattern

# ---- change these ----------------------------------------------------------
PERIOD = 1.0                    # Faster period (200ms) for a strobe effect
COLORS = [
    ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),   # red
    ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),   # blue
]
# ----------------------------------------------------------------------------

# The Duckiebot has 5 LED slots, in this order:
#   0 left front, 1 right rear, 2 right front, 3 unused, 4 left rear
LED_COUNT = 5


class Blinker(Node):
    def __init__(self, vehicle_name):
        super().__init__('blinker')
        self.publisher = self.create_publisher(
            LEDPattern, f'/{vehicle_name}/led_pattern', 1)
        self.index = 0
        self.timer = self.create_timer(PERIOD, self.publish_pattern)
        self.get_logger().info(f'Blinking police lights on {vehicle_name} every {PERIOD}s')

    def publish_pattern(self):
        color = COLORS[self.index % len(COLORS)]

        msg = LEDPattern()
        msg.rgb_vals = [color] * LED_COUNT
        self.publisher.publish(msg)

        self.index += 1


def main():
    vehicle_name = os.environ.get('VEHICLE_NAME')
    if not vehicle_name:
        raise SystemExit('VEHICLE_NAME is not set. Run: export VEHICLE_NAME=duckie03')

    rclpy.init()
    node = Blinker(vehicle_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
