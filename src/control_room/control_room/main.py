#!/usr/bin/python3
"""Drive the Duckiebot with the arrow keys. No GUI, terminal only.

This node reads single key presses and publishes a short command string to
/<USER_NAME>/<VEHICLE_NAME>/command. It never talks to the robot hardware
directly - that is the job of the "robot" node, which listens on the same
topic.

    control_room  --(command)-->  robot  --(wheels_cmd, led_pattern)-->  Duckiebot

Keys:
    arrow up    / w     forward
    arrow down  / s     backward
    arrow left  / a     turn left
    arrow right / d     turn right
    space               stop
    1 / 2 / 3           LEDs red / green / blue
    0                   LEDs white
    q                   quit (sends stop first)
"""
import os
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ---- change these ----------------------------------------------------------
# key -> command string
KEY_MAP = {
    'UP': 'f',    'w': 'f',
    'DOWN': 'b',  's': 'b',
    'LEFT': 'l',  'a': 'l',
    'RIGHT': 'r', 'd': 'r',
    ' ': 's',
    '1': 'rl',
    '2': 'gl',
    '3': 'bl',
    '0': 'sol',
}
QUIT_KEY = 'q'
# ----------------------------------------------------------------------------

ARROWS = {b'A': 'UP', b'B': 'DOWN', b'C': 'RIGHT', b'D': 'LEFT'}


class ControlRoom(Node):
    def __init__(self, user, vehicle_name):
        super().__init__('control_room')
        self.publisher = self.create_publisher(
            String, f'/{user}/{vehicle_name}/command', 10)
        self.get_logger().info(
            'Keyboard ready. Arrows or WASD to drive, space to stop, q to quit.')

    def send(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent: {command}')


class KeyReader:
    """Reads one key press at a time from the terminal.

    Opens /dev/tty instead of stdin, so it also works when the node is
    started by "ros2 launch" and stdin is not the keyboard.
    """

    def __init__(self):
        try:
            self.fd = os.open('/dev/tty', os.O_RDONLY)
        except OSError:
            raise SystemExit('No keyboard here. Start this node in a terminal '
                             'you can type into.')
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def close(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
        os.close(self.fd)

    def read_key(self):
        first = os.read(self.fd, 1)
        if first != b'\x1b':
            return first.decode(errors='ignore')

        if not select.select([self.fd], [], [], 0.05)[0]:
            return 'ESC'

        second = os.read(self.fd, 1)
        if second != b'[':
            return second.decode(errors='ignore')

        return ARROWS.get(os.read(self.fd, 1), 'ESC')


def main():
    vehicle_name = os.environ.get('VEHICLE_NAME')
    user = os.environ.get('USER_NAME')
    if not vehicle_name:
        raise SystemExit('VEHICLE_NAME is not set. Run: export VEHICLE_NAME=duckie03')
    if not user:
        raise SystemExit('USER_NAME is not set. Run: export USER_NAME=your_name')

    keys = KeyReader()
    rclpy.init()
    node = ControlRoom(user, vehicle_name)

    try:
        while rclpy.ok():
            key = keys.read_key()
            if len(key) == 1:
                key = key.lower()

            if key == QUIT_KEY:
                break
            if key in KEY_MAP:
                node.send(KEY_MAP[key])
    except KeyboardInterrupt:
        pass
    finally:
        keys.close()
        if rclpy.ok():
            node.send('s')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
