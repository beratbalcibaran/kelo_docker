import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from kelo_bringup.logic import quat_from_yaw


def log(msg):
    print(f'[send_goal] {msg}', flush=True)


def localization_ok(node, wait_s=1.5):
    state = []
    node.create_subscription(String, '/kelo/localization_state', lambda m: state.append(m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    end = time.time() + wait_s
    while time.time() < end and not state:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not state:
        log('WARNING: no localization state (localize did not run); assuming the AMCL pose is correct.')
        return True
    if state[-1] != 'LOCALIZED':
        log(f'Goal NOT sent: localization state {state[-1]} (run "localize" / GUI "Localize" first).')
        return False
    return True


def main():
    if len(sys.argv) < 3:
        log('Usage: send_goal <x> <y> [yaw_deg]')
        sys.exit(1)
    x, y = float(sys.argv[1]), float(sys.argv[2])
    yaw = math.radians(float(sys.argv[3]) if len(sys.argv) > 3 else 0.0)

    rclpy.init()
    node = Node('send_goal')
    if not localization_ok(node):
        rclpy.shutdown()
        sys.exit(5)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    log('Waiting for Nav2 (navigate_to_pose)...')
    t0 = time.time()
    while not client.wait_for_server(timeout_sec=2.0):
        log(f'Nav2 not ready yet ({int(time.time() - t0)} s). Was a start pose given? '
            f'(GUI "Localize" / "Set Initial Pose" or RViz "2D Pose Estimate")')
        if time.time() - t0 > 180:
            log('Timeout: Nav2 did not start.')
            rclpy.shutdown()
            sys.exit(2)

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    _, _, goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = quat_from_yaw(yaw)

    last = [0.0]

    def feedback(fb):
        if time.time() - last[0] > 2.0:
            last[0] = time.time()
            log(f'Remaining distance: {fb.feedback.distance_remaining:.2f} m')

    log(f'Driving to goal: x={x} y={y} yaw={math.degrees(yaw):.1f} deg')
    send = client.send_goal_async(goal, feedback_callback=feedback)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if handle is None or not handle.accepted:
        log('Goal REJECTED (Nav2 not active or invalid goal).')
        rclpy.shutdown()
        sys.exit(3)
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result)
    status = result.result().status
    names = {GoalStatus.STATUS_SUCCEEDED: 'Goal reached.', GoalStatus.STATUS_CANCELED: 'Cancelled.',
             GoalStatus.STATUS_ABORTED: 'Failed (aborted): no path found or the controller gave up; '
                                        'check the [navigation] logs.'}
    log(names.get(status, f'Unknown result: {status}'))
    rclpy.shutdown()
    sys.exit(0 if status == GoalStatus.STATUS_SUCCEEDED else 4)


if __name__ == '__main__':
    main()
