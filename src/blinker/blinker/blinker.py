#!/usr/bin/python3
"""Show white LEDs until a command event changes them to green.

Publishes to /<VEHICLE_NAME>/led_pattern.
"""
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String

from duckietown_msgs.msg import LEDPattern

# ---- LED states ------------------------------------------------------------
no_event = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
color_detected = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
on_finish = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
# ----------------------------------------------------------------------------
topic_name = 'LED_commands'
# The Duckiebot has 5 LED slots, in this order:
#   0 left front, 1 right rear, 2 right front, 3 unused, 4 left rear
LED_COUNT = 5


class Blinker(Node):
    def __init__(self, user, vehicle_name):
        super().__init__('blinker')
        self.publisher = self.create_publisher(
            LEDPattern, f'/{vehicle_name}/led_pattern', 1)
        self.command_subscription = self.create_subscription(
            String,
            f'/{user}/{vehicle_name}/${topic_name}',
            self.on_command,
            10,
        )
        self.publish_color(no_event)
        self.get_logger().info(f'Waiting for an event on {user}/{vehicle_name}/command')

    def on_command(self, msg):
        event=msg.data
        if event == 'nan':
            self.publish_color(no_event)
            self.get_logger().info(f'Event received: {msg.data}; LEDs set to white')

        elif event == 'bottle':
            self.publish_color(color_detected)
            self.get_logger().info(f'Event received: {msg.data}; LEDs set to red')

        elif event == 'finished':
            self.publish_color(on_finish)
            self.get_logger().info(f'Event received: {msg.data}; LEDs set to green')

        else:
            self.get_logger().warn(f'Unknown event received: {msg.data}')

    def publish_color(self, color):
        msg = LEDPattern()
        msg.rgb_vals = [color] * LED_COUNT
        self.publisher.publish(msg)


def main():
    vehicle_name = os.environ.get('VEHICLE_NAME')
    user = os.environ.get('USER_NAME')
    if not vehicle_name:
        raise SystemExit('VEHICLE_NAME is not set. Run: export VEHICLE_NAME=duckie03')
    if not user:
        raise SystemExit('USER_NAME is not set. Run: export USER_NAME=your_name')

    rclpy.init()
    node = Blinker(user, vehicle_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

