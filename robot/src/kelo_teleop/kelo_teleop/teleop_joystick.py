#!/usr/bin/env python3
"""Joystick teleop for the KELO ROBILE, with 4 selectable speed levels.

Replaces teleop_twist_joy (fixed scales and turbo only) to cycle 25/50/75/100% of the maximum speed
with one button. Reads /joy (from joy_node) and publishes geometry_msgs/Twist on /cmd_vel, read by
kelo_tulip_ros2 (platform_driver_node).

Xbox Series X controller (joy_node with the standard xpad driver):
  axis 1 (LY)     -> forward/backward (vx)
  axis 3 (RX)     -> rotation (va)
  button 4 (LB)   -> deadman switch: hold it to move
  button 0 (A)    -> cycles the speed level (25% -> 50% -> 75% -> 100% -> 25% ...)
  button 5 (RB)   -> safety stop: forces zero velocity, highest priority (also with LB held and the
                     sticks moved)

For another joystick layout set the ROS parameters (see __init__).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

SPEED_LEVELS = [0.25, 0.50, 0.75, 1.00]

# Maximum speeds (100%) = the driver's default software limits (PlatformDriver.cpp: maxvlin=1.5,
# maxva=1.0), not the mechanical limit of the motors (about 5.6 m/s per hub wheel).
LIN_MAX = 1.5
ANG_MAX = 1.0


class TeleopJoystick(Node):
    def __init__(self):
        super().__init__('kelo_teleop_joystick')

        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 3)
        self.declare_parameter('enable_button', 4)
        self.declare_parameter('speed_button', 0)
        self.declare_parameter('stop_button', 5)
        self.declare_parameter('start_level', 0)  # index in SPEED_LEVELS, 0 = 25%

        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.enable_button = self.get_parameter('enable_button').value
        self.speed_button = self.get_parameter('speed_button').value
        self.stop_button = self.get_parameter('stop_button').value
        self._stop_button_prev = 0

        self.level_idx = max(0, min(len(SPEED_LEVELS) - 1, self.get_parameter('start_level').value))
        self._speed_button_prev = 0

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Joy, '/joy', self._on_joy, 10)

        self.get_logger().info(
            f"Joystick teleop started. Speed level: {int(SPEED_LEVELS[self.level_idx] * 100)}% "
            f"(button {self.speed_button} changes it, button {self.enable_button} = deadman, "
            f"button {self.stop_button} = safety stop)."
        )

    def _on_joy(self, msg: Joy):
        # Safety stop: highest priority, checked first. Forces zero velocity at once and looks at
        # nothing else in this cycle.
        stop_button_now = msg.buttons[self.stop_button] if len(msg.buttons) > self.stop_button else 0
        if stop_button_now == 1:
            if self._stop_button_prev == 0:
                self.get_logger().warn("Safety STOP (RB): velocity forced to zero.")
            self._stop_button_prev = stop_button_now
            self.pub.publish(Twist())
            return
        self._stop_button_prev = stop_button_now

        # Speed level changes on the rising edge of the button only (otherwise it would keep cycling
        # while the button is held).
        speed_button_now = msg.buttons[self.speed_button] if len(msg.buttons) > self.speed_button else 0
        if speed_button_now == 1 and self._speed_button_prev == 0:
            self.level_idx = (self.level_idx + 1) % len(SPEED_LEVELS)
            self.get_logger().info(f"Speed: {int(SPEED_LEVELS[self.level_idx] * 100)}%")
        self._speed_button_prev = speed_button_now

        enabled = len(msg.buttons) > self.enable_button and msg.buttons[self.enable_button] == 1

        twist = Twist()
        if enabled:
            scale = SPEED_LEVELS[self.level_idx]
            lin = msg.axes[self.axis_linear] if len(msg.axes) > self.axis_linear else 0.0
            ang = msg.axes[self.axis_angular] if len(msg.axes) > self.axis_angular else 0.0
            twist.linear.x = lin * LIN_MAX * scale
            twist.angular.z = ang * ANG_MAX * scale
        # Without the deadman the twist stays zero.

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # final stop
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
