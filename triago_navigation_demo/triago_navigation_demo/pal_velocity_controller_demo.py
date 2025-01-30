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

    def send_goal(self, a, b):
        self._is_successful = None
        goal_msg = PlayMotion2.Goal()
        goal_msg.motion_name = a
        goal_msg.skip_planning = b
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
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
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

    def send_vel_goal(self, a, b, c, d, e):
        self._is_vel_successful = None
        goal_msg = LimitedTimeDriving.Goal()
        goal_msg.linear_vel_x = a
        goal_msg.linear_vel_y = b
        goal_msg.angular_vel = c
        goal_msg.duration = d
        goal_msg.topic_name = e
        self._vel_action_client.wait_for_server()
        future = self._vel_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.vel_goal_response_callback)
        rclpy.spin_until_future_complete(self, future)
        while self._is_vel_successful is None:
            time.sleep(1.0)
            rclpy.spin_once(self)
        if not self._is_vel_successful:
            self.get_logger().error("Action failed")
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
            self.get_logger().info("Limited Time Driving succeeded")
        else:
            self._is_vel_successful = False
            self.get_logger().error("Limited Time Driving failed with error {}".format(error))



def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerDemo()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    node._action_client.wait_for_server()
    node._vel_action_client.wait_for_server()

    node.send_goal('home', False)
    node.send_goal('torso_down', False)

    node.get_logger().info('Starting first navigation routine.')
    node.send_vel_goal(0.5, 0.0, 0.0, 2.0, '/cmd_vel')
    node.send_vel_goal(0.0, 0.5, 0.0, 2.0, '/cmd_vel')
    node.send_vel_goal(0.0, 0.0, 0.35, 2.0, '/cmd_vel')

    node.send_goal('torso_up', False)
    node.send_vel_goal(0.0, 0.0, -0.35, 2.0, '/cmd_vel')
    node.send_vel_goal(0.0, -0.5, 0.0, 2.0, '/cmd_vel')
    node.send_vel_goal(-0.5, 0.0, 0.0, 2.0, '/cmd_vel')

    node.get_logger().info('Path cleared after navigation.')

    rclpy.spin_once(node)
    executor.shutdown()
    executor_thread.join()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()