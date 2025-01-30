#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.msg import SpeedLimit
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from play_motion2_msgs.action import PlayMotion2
import rcl_interfaces
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import threading
import subprocess
import signal


class CartesianNavigationDemo(Node):

    def __init__(self):
        super().__init__('pal_cartesian_navigation_demo')
        self._action_client = ActionClient(self, PlayMotion2, '/play_motion2')
        self._is_successful = None
        self._max_velocity_param = rcl_interfaces.msg.Parameter()
        self._speed_publisher = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.launch_process = None
        self.launch_thread = None

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

    def speed_limit_msg(self, max_vel):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.speed_limit = max_vel
        return msg

    def start_periodic_move(self):
        '''Start the launch file in a separate thread.'''

        def run_launch():
            launch_command = [
                'ros2',
                'launch',
                'periodic_move_joint',
                'periodic_move_joint.launch.py',
                'config:=triago_navigation/triago_navigation_demo/triago_navigation_demo/triago_full_body.yaml',
            ]
            self.get_logger().info(f"Starting launch file: {' '.join(launch_command)}")
            self.launch_process = subprocess.Popen(
                launch_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            self.launch_process.wait()  # Wait for the process to complete

        self.launch_thread = threading.Thread(target=run_launch)
        self.launch_thread.start()

    def stop_periodic_move(self):
        '''Stop the launch file by sending a SIGINT signal.'''
        if self.launch_process is not None:
            self.get_logger().info('Stopping periodic move joints...')
            self.launch_process.send_signal(signal.SIGINT)
            self.launch_process.wait()  # Wait for clean shutdown
            self.get_logger().info('MoveJoint Launch file stopped.')
            self.launch_process = None

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
