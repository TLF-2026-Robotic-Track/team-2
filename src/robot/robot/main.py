#!/usr/bin/python3
"""Movements: turn command strings into real wheel and LED commands.

Listens on /<USER_NAME>/<VEHICLE_NAME>/command and publishes to
/<VEHICLE_NAME>/wheels_cmd and /<VEHICLE_NAME>/led_pattern.

The commands come from the control_room node (arrow keys), but you can also
send them by hand:

    ros2 topic pub --once /$USER_NAME/$VEHICLE_NAME/command \\
        std_msgs/msg/String "{data: 'f'}"

Wheel speeds latch: after a "forward" the robot keeps driving until it gets
another command. Send 's' (space in control_room) to stop.
"""
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header, String

from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped

# ---- change these ----------------------------------------------------------
SPEEDL = 1.0       # forward / backward speed, from 0.0 to 1.0
SPEEDR = 0.9
TURN_SPEED = 0.5    # wheel speed while turning on the spot

# command -> (left wheel, right wheel)
WHEELS = {
    'f': (SPEEDL, SPEEDR),                # forward
    'b': (-SPEEDL, -SPEEDR),              # backward
    'l': (-TURN_SPEED, TURN_SPEED),     # turn left on the spot
    'r': (TURN_SPEED, -TURN_SPEED),     # turn right on the spot
    's': (0.0, 0.0),                    # stop
}

# command -> LED color
LIGHTS = {
    'rl': ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
    'gl': ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
    'bl': ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
    'sol': ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
}
# ----------------------------------------------------------------------------

LED_COUNT = 5


class Robot(Node):
    def __init__(self, user, vehicle_name):
        super().__init__('robot')

        self.create_subscription(
            String, f'/{user}/{vehicle_name}/command', self.on_command, 10)

        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped, f'/{vehicle_name}/wheels_cmd', 10)
        self.led_pub = self.create_publisher(
            LEDPattern, f'/{vehicle_name}/led_pattern', 10)

        self.get_logger().info(f'Waiting for commands for {vehicle_name}...')

    def on_command(self, msg):
        command = msg.data.lower()

        if command in WHEELS:
            left, right = WHEELS[command]
            self.get_logger().info(f'{command}: wheels {left} {right}')
            self.run_wheels(command, left, right)
        elif command in LIGHTS:
            self.get_logger().info(f'{command}: lights')
            self.set_lights(LIGHTS[command])
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def run_wheels(self, name, vel_left, vel_right):
        msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = name
        msg.header = header
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        self.wheels_pub.publish(msg)

    def set_lights(self, color):
        msg = LEDPattern()
        msg.rgb_vals = [color] * LED_COUNT
        self.led_pub.publish(msg)


def main():
    vehicle_name = os.environ.get('VEHICLE_NAME')
    user = os.environ.get('USER_NAME')
    if not vehicle_name:
        raise SystemExit('VEHICLE_NAME is not set. Run: export VEHICLE_NAME=duckie03')
    if not user:
        raise SystemExit('USER_NAME is not set. Run: export USER_NAME=your_name')

    rclpy.init()
    node = Robot(user, vehicle_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.run_wheels('stop', 0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
