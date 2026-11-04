#!/usr/bin/env python3
"""Keyboard teleop for the KELO ROBILE (omnidirectional platform).

Publishes geometry_msgs/Twist on /cmd_vel, read by kelo_tulip_ros2 (platform_driver_node). Reads the
keys with termios/tty, so it also works over SSH without X11. Keep a key pressed to keep moving: the
driver stops the robot after cmd_vel_timeout_s without commands.
"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

INSTRUCTIONS = """
KELO ROBILE keyboard teleop (omnidirectional base)

   q    w    e
   a    s    d
   z         c

  w/z : forward / backward       (vx)
  a/d : sideways left / right    (vy)
  q/e : turn left / right        (va)
  s   : immediate STOP
  x   : faster
  c   : slower

CTRL-C to quit (always stops the robot first)
"""

MOVE_BINDINGS = {
    'w': (1, 0, 0),
    'z': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
}

SPEED_STEP = 0.05
LIN_MAX = 0.6
ANG_MAX = 0.8


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('kelo_teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lin_speed = 0.2
        self.ang_speed = 0.3

    def publish_twist(self, vx, vy, va):
        msg = Twist()
        msg.linear.x = vx * self.lin_speed
        msg.linear.y = vy * self.lin_speed
        msg.angular.z = va * self.ang_speed
        self.pub.publish(msg)

    def stop(self):
        self.publish_twist(0.0, 0.0, 0.0)


def main():
    settings = termios.tcgetattr(sys.stdin.fileno())
    rclpy.init()
    node = TeleopKeyboard()
    print(INSTRUCTIONS)
    print(f'Current speed: lin={node.lin_speed:.2f} m/s  ang={node.ang_speed:.2f} rad/s')

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key in MOVE_BINDINGS:
                vx, vy, va = MOVE_BINDINGS[key]
                node.publish_twist(vx, vy, va)
            elif key == 's':
                node.stop()
            elif key == 'x':
                node.lin_speed = min(LIN_MAX, node.lin_speed + SPEED_STEP)
                node.ang_speed = min(ANG_MAX, node.ang_speed + SPEED_STEP)
                node.stop()
                print(f'Speed: lin={node.lin_speed:.2f} m/s  ang={node.ang_speed:.2f} rad/s')
            elif key == 'c':
                node.lin_speed = max(0.0, node.lin_speed - SPEED_STEP)
                node.ang_speed = max(0.0, node.ang_speed - SPEED_STEP)
                node.stop()
                print(f'Speed: lin={node.lin_speed:.2f} m/s  ang={node.ang_speed:.2f} rad/s')
            elif key == '\x03':  # CTRL-C
                break
            else:
                # unknown or released key: stop
                node.stop()
    except Exception as e:
        print(e)
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
