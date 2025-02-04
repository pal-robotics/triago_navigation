#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.msg import SpeedLimit
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from play_motion2_msgs.action import PlayMotion2
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyPriority
from diagnostic_msgs.msg import DiagnosticArray

import threading
import subprocess
import signal


class CartesianNavigationDemo(Node):

    def __init__(self):
        super().__init__('pal_cartesian_navigation_demo')
        self._action_client = ActionClient(self, PlayMotion2, '/play_motion2')
        self._is_successful = None
        self._start_navigation = False
        self._switch_joy = None

        self._speed_publisher = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self._launch_process = None
        self._launch_thread = None

        # ASSISTED TELEOP HANDLING #
        self._teleop_subscription = self.create_subscription(
            Bool,
            '/assisted_teleop_priority',
            self.teleop_sub_callback,
            10)
        self._teleop_client = ActionClient(self, JoyPriority, '/assisted_teleop_priority_action')
        self._teleop_successful_toggle = None

        # JOY HANDLING #
        self._diagnostics_subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics_agg',
            self.diagnostics_callback,
            10)
        self._joy_client = ActionClient(self, JoyPriority, '/joy_priority_action')
        self._joy_successful_toggle = None

    # PLAY MOTION2 #
    def send_goal(self, motion, skip_plan):
        self._is_successful = None
        goal_msg = PlayMotion2.Goal()
        goal_msg.motion_name = motion
        goal_msg.skip_planning = skip_plan
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self, future)
        while self._is_successful is None:
            time.sleep(1.0)
            rclpy.spin_once(self)
        if not self._is_successful:
            self.get_logger().error('Action failed')
            return

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._is_successful = False
            self.get_logger().error('PlayMotion2 goal rejected')
            return

        self.get_logger().info('PlayMotion2 goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        error = result.error
        if error == '':
            self._is_successful = True
            self.get_logger().info('Motion succeeded')
        else:
            self._is_successful = False
            self.get_logger().error('Motion failed with error {}'.format(error))

    # ASSISTED TELEOP TOGGLING PRIORITY #
    def teleop_sub_callback(self, msg):
        self.get_logger().info(f"Teleop toggle resulting message: {msg.data}")
        self._teleop_priority = msg.data

    def teleop_goal(self):
        self._teleop_successful_toggle = None
        goal = JoyPriority.Goal()
        self._teleop_client.wait_for_server()

        future = self._teleop_client.send_goal_async(goal)
        future.add_done_callback(self.teleop_goal_response_callback)
        rclpy.spin_until_future_complete(self, future)
        while self._teleop_successful_toggle is None:
            rclpy.spin_once(self)
        if not self._teleop_successful_toggle:
            self.get_logger().error('Teleop action failed')
            return

    def teleop_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._teleop_successful_toggle = False
            self.get_logger().error('Teleop goal rejected')
            return

        self.get_logger().info('Teleop goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.teleop_get_result_callback)

    def teleop_get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._teleop_successful_toggle = True
            self.get_logger().info('Teleop succeeded')
        else:
            self._teleop_successful_toggle = False
            self.get_logger().info(f"Teleop failed with status: {status}")

    # JOY TOGGLING PRIORITY
    def diagnostics_callback(self, msg):
        for status in msg.status:
            for value in status.values:
                if value.key == 'current priority':
                    if value.value == '100':
                        self.get_logger().info('Priority is on joystick, switching')
                        self._switch_joy = True
                    else:
                        self._switch_joy = False
                        return

    def joy_goal(self):
        self._joy_successful_toggle = None
        goal = JoyPriority.Goal()
        self._joy_client.wait_for_server()

        future = self._joy_client.send_goal_async(goal)
        future.add_done_callback(self.joy_goal_response_callback)
        rclpy.spin_until_future_complete(self, future)
        while self._joy_successful_toggle is None:
            rclpy.spin_once(self)
        if not self._joy_successful_toggle:
            self.get_logger().error('Joy action failed')
            return

    def joy_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._joy_successful_toggle = False
            self.get_logger().error('Joy goal rejected')
            return

        self.get_logger().info('Joy goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.joy_get_result_callback)

    def joy_get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._joy_successful_toggle = True
            self.get_logger().info('Joy succeeded')
            self._start_navigation = True
        else:
            self._joy_successful_toggle = False
            self.get_logger().info(f"Joy failed with status: {status}")

    def speed_limit_msg(self, max_vel):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.speed_limit = max_vel
        return msg

    def start_periodic_move(self):
        # Start the launch file in a separate thread. #

        def run_launch():
            launch_command = [
                'ros2',
                'launch',
                'periodic_move_joint',
                'periodic_move_joint.launch.py',
                'config:=triago_navigation/triago_navigation_demo/triago_navigation_demo/'
                'triago_full_body.yaml',
            ]
            self.get_logger().info(f"Starting launch file: {' '.join(launch_command)}")
            self._launch_process = subprocess.Popen(
                launch_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            self._launch_process.wait()  # Wait for the process to complete

        self._launch_thread = threading.Thread(target=run_launch)
        self._launch_thread.start()

    def stop_periodic_move(self):
        # Stop the launch file by sending a SIGINT signal. #
        if self._launch_process is not None:
            self.get_logger().info('Stopping periodic move joints...')
            self._launch_process.send_signal(signal.SIGINT)
            self._launch_process.wait()  # Wait for clean shutdown
            self.get_logger().info('MoveJoint Launch file stopped.')
            self._launch_process = None

    def navigation_routine(self, i):
        goal = PoseStamped()
        goal.header.frame_id = 'base_footprint'
        goal.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.position.x = 1.0

        path = Path()
        path.header.frame_id = 'base_footprint'
        path.header.stamp = rclpy.time.Time().to_msg()
        path.poses.append(goal)

        navigator = BasicNavigator(node_name='navigator_{}'.format(i + 1))
        navigator.followPath(path, 'TrackPose', 'general_goal_checker')
        navigator.waitUntilNav2Active()
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')

        path.poses.clear()
        navigator.destroyNode()

        goal.pose.position.x = 0.0
        goal.pose.position.y = 1.0
        path.poses.append(goal)
        navigator = BasicNavigator(node_name='navigator_{}'.format(i + 2))
        navigator.followPath(path, 'TrackPose', 'general_goal_checker')
        navigator.waitUntilNav2Active()
        path.poses.clear()
        navigator.destroyNode()

        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.orientation.z = -1.0
        goal.pose.orientation.w = 0.0

        path.poses.append(goal)
        navigator = BasicNavigator(node_name='navigator_{}'.format(i + 3))

        navigator.followPath(path, 'TrackPose', 'general_goal_checker')
        navigator.waitUntilNav2Active()
        path.poses.clear()
        navigator.destroyNode()

        goal.pose.position.x = 1.0
        goal.pose.position.y = 1.0
        goal.pose.orientation.z = -1.0
        goal.pose.orientation.w = 0.0

        path.poses.append(goal)
        navigator = BasicNavigator(node_name='navigator_{}'.format(i + 4))

        navigator.followPath(path, 'TrackPose', 'general_goal_checker')
        navigator.waitUntilNav2Active()
        path.poses.clear()
        navigator.destroyNode()


def main(args=None):
    rclpy.init(args=args)
    node = CartesianNavigationDemo()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    node._action_client.wait_for_server()
    node._teleop_client.wait_for_server()
    node._joy_client.wait_for_server()

    node.get_logger().info('Checking assisted teleop priority')
    node.teleop_goal()

    if node._teleop_priority:
        node.get_logger().info('Assisted Teleop priority is enabled, switching it off')
        node.teleop_goal()
    else:
        node.get_logger().info('Assisted Teleop priority is not enabled')

    while node._switch_joy is None:
        rclpy.spin_once(node)
    if node._switch_joy:
        node.joy_goal()
    else:
        node.get_logger().info('Joy Priority is on autonomous, no need to switch')
        node._start_navigation = True

    if node._start_navigation:
        node.get_logger().info('Starting pal_cartesian_navigation Demo.')

        node.get_logger().info('Motion HOME starting.')
        node.send_goal('home', False)
        node.get_logger().info('Motion torso_down starting.')
        node.send_goal('torso_down', False)
        node.get_logger().info('Start navigating.')
        node.navigation_routine(0)

        node.get_logger().info('Setting velocity to half of the maximum value.')
        node._speed_publisher.publish(node.speed_limit_msg(0.4))
        node.get_logger().info('Motion horizontal starting.')
        node.send_goal('horizontal', False)
        node.get_logger().info('Starting moving arms while navigating.')
        node.start_periodic_move()
        time.sleep(10)
        node.navigation_routine(4)
        node.stop_periodic_move()
        time.sleep(10)

        node.get_logger().info('Setting full velocity.')
        node._speed_publisher.publish(node.speed_limit_msg(0.0))
        node.get_logger().info('Motion HOME starting.')
        node.send_goal('home', False)
        node.get_logger().info('Motion torso_up starting.')
        node.send_goal('torso_up', False)
        node.get_logger().info('Start navigating.')
        node.navigation_routine(8)

        node.get_logger().info('Setting velocity to half of the maximum value.')
        node._speed_publisher.publish(node.speed_limit_msg(0.4))
        node.get_logger().info('Motion horizontal starting.')
        node.send_goal('horizontal', False)
        node.get_logger().info('Starting moving arms while navigating.')
        node.start_periodic_move()
        time.sleep(10)
        node.navigation_routine(12)
        node.stop_periodic_move()
        time.sleep(5)

        node.get_logger().info('Motion HOME starting.')
        node.send_goal('home', False)
        node.get_logger().info('Demo completed!')

    rclpy.spin_once(node)
    executor.shutdown()
    executor_thread.join()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
