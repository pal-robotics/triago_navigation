#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from play_motion2_msgs.action import PlayMotion2
import rcl_interfaces
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from pal_nav2_msgs.action import LimitedTimeDriving

import threading


class VelocityControllerDemo(Node):

    def __init__(self):
        super().__init__('pal_velocity_controller_demo')
        self._action_client = ActionClient(self, PlayMotion2, '/play_motion2')
        self._is_successful = None
        self._vel_action_client = ActionClient(self, LimitedTimeDriving, '/limited_time_driving')
        self._is_vel_successful = None

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
            self.get_logger().error("Action failed")
            return

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._is_successful = False
            self.get_logger().error("PlayMotion2 goal rejected")
            return

        self.get_logger().info("PlayMotion2 goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        error = result.error
        if error == "":
            self._is_successful = True
            self.get_logger().info("Motion succeeded")
        else:
            self._is_successful = False
            self.get_logger().error("Motion failed with error {}".format(error))

    # Limited Velocity Driving Handling

    def send_vel_goal(self, vel_x, vel_y, ang_vel, d):
        self._is_vel_successful = None
        goal_msg = LimitedTimeDriving.Goal()
        goal_msg.linear_vel_x = vel_x
        goal_msg.linear_vel_y = vel_y
        goal_msg.angular_vel = ang_vel
        goal_msg.duration = d
        self._vel_action_client.wait_for_server()
        future = self._vel_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.vel_goal_response_callback)
        rclpy.spin_until_future_complete(self, future)
        while self._is_vel_successful is None:
            time.sleep(1.0)
            rclpy.spin_once(self)
        if not self._is_vel_successful:
            self.get_logger().error("Speed Publishing failed")
            return
    
    def vel_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._is_vel_successful = False
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_vel_result_callback)

    def get_vel_result_callback(self, future):
        result = future.result().result
        error = result.error_code
        if error == 0:
            self._is_vel_successful = True
        else:
            self._is_vel_successful = False
            self.get_logger().error("Speed Publishing failed with error {}".format(error))



def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerDemo()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    node._action_client.wait_for_server()
    node._vel_action_client.wait_for_server()
 
    node.get_logger().info('Starting pal_velocity_controller Demo.')

    node.get_logger().info('Motion HOME starting.')
    node.send_goal('home', False)
    node.get_logger().info('Motion torso_down starting.')
    node.send_goal('torso_down', False)

    node.get_logger().info('Start navigating.')
    node.send_vel_goal(0.5, 0.0, 0.0, 2.0)
    node.send_vel_goal(0.0, 0.5, 0.0, 2.0)
    node.send_vel_goal(0.0, 0.0, 0.35, 2.0)

    node.get_logger().info('Motion torso_up starting.')
    node.send_goal('torso_up', False)
    node.get_logger().info('Start navigating.')
    node.send_vel_goal(0.0, 0.0, -0.35, 2.0)
    node.send_vel_goal(0.0, -0.5, 0.0, 2.0)
    node.send_vel_goal(-0.5, 0.0, 0.0, 2.0)

    node.get_logger().info('Path cleared after navigation.')

    rclpy.spin_once(node)
    executor.shutdown()
    executor_thread.join()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()