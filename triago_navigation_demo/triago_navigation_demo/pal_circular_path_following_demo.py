#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from twist_mux_msgs.action import JoyPriority
from action_msgs.msg import GoalStatus
import threading


class CircularPathFollowingDemo(Node):

    def __init__(self):
        super().__init__('pal_circular_path_following_demo')
        self.subscription = self.create_subscription(
          Odometry,
          '/mobile_base_controller/odom',
          self.odom_callback,
          10)
        self.first_time = True
        self.start = None
        self.start_navigation = False
        
        # ASSISTED TELEOP HANDLING
        self.teleop_priority = None
        self.teleop_subscription = self.create_subscription(
            Bool,
            '/assisted_teleop_priority',
            self.teleop_sub_callback,
            10)
        self.teleop_client = ActionClient(self, JoyPriority, '/assisted_teleop_priority_action')
        self.teleop_successful_toggle = None
        
        # JOY HANDLING
        self.joy_priority = None 
        self.joy_subscription = self.create_subscription(
            Bool,
            '/joy_priority',
            self.joy_sub_callback,
            10)
        self.joy_client = ActionClient(self, JoyPriority, '/joy_priority_action')
        self.joy_successful_toggle = None
    
    # ASSISTED TELEOP TOGGLING PRIORITY
    def teleop_sub_callback(self, msg):
        self.get_logger().info(f"Teleop callback received message: {msg.data}")
        if msg.data:
            self.get_logger().info('Teleop priority enabled')
            self.teleop_priority = True
        else:
            self.get_logger().info('Teleop priority disabled')
            self.teleop_priority = False

    def teleop_goal(self):
        self.teleop_successful_toggle = None
        goal = JoyPriority.Goal()
        self.teleop_client.wait_for_server()

        future = self.teleop_client.send_goal_async(goal)
        future.add_done_callback(self.teleop_goal_response_callback)
        rclpy.spin_until_future_complete(self, future)
        while self.teleop_successful_toggle is None:
            rclpy.spin_once(self)
        if not self.teleop_successful_toggle:
            self.get_logger().error("Action failed")
            return
        
    def teleop_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.teleop_successful_toggle = False
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.teleop_get_result_callback)

    def teleop_get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.teleop_successful_toggle = True
            self.get_logger().info("Teleop succeeded")
        else:
            self.teleop_successful_toggle = False
            self.get_logger().info(f"Teleop failed with status: {status}")
        
    # JOY TOGGLING PRIORITY
    def joy_sub_callback(self, msg):
        self.get_logger().info(f"Joy callback received message: {msg.data}")
        if msg.data:
            self.get_logger().info('Joy priority enabled')
            self.joy_priority = True
        else:
            self.get_logger().info('Joy priority disabled')
            self.teleop_priority = False
    
    def joy_goal(self):
        self.joy_successful_toggle = None
        goal = JoyPriority.Goal()
       
        self.joy_client.wait_for_server()

        future = self.joy_client.send_goal_async(goal)
        future.add_done_callback(self.joy_goal_response_callback)
        rclpy.spin_until_future_complete(self, future)
        while self.joy_successful_toggle is None:
            rclpy.spin_once(self)
        if not self.joy_successful_toggle:
            self.get_logger().error("Action failed")
            return
    
    def joy_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.joy_successful_toggle = False
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.joy_get_result_callback)
    
    def joy_get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.joy_successful_toggle = True
            self.get_logger().info("Joy succeeded")
            self.start_navigation = True
        else:
            self.teleop_successful_toggle = False
            self.get_logger().info(f"Joy failed with status: {status}")
       
    def odom_callback(self, msg):
        if self.first_time:
            self.start = PoseStamped()
            self.start.header.stamp = msg.header.stamp
            self.start.pose = msg.pose.pose
            self.first_time = False
            

def main(args=None):
    rclpy.init(args=args)
    node = CircularPathFollowingDemo()
    navigator = BasicNavigator()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    while node.first_time:
        rclpy.spin_once(node)

    start = node.start
    node.get_logger().info('Odometry start point initialized.')
    
    node.teleop_client.wait_for_server()
    node.joy_client.wait_for_server()

    if node.teleop_priority:
        node.get_logger().info('Assisted Teleop priority is enabled')
        node.teleop_goal()
    else:
        node.get_logger().info('Assisted Teleop priority is not enabled')
        node.teleop_goal()
    if node.joy_priority:
        node.get_logger().info('Joy priority is enabled')
        node.joy_goal()
    else:
        node.get_logger().info('Joy priority is not enabled')
        node.joy_goal()

    if node.start_navigation:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = node.get_clock().now().to_msg()
    
        path = navigator.getPath(start, goal, 'FunctionPlanner', False)
        node.get_logger().info('Path received, following it.')
    
        navigator.followPath(path, 'TrackPose', 'general_goal_checker')
        navigator.waitUntilNav2Active()
    
        rclpy.spin_once(node)
    
        if navigator.getResult() == 'Succeeded':
            node.get_logger().info('Task succeeded')
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
    
        path.poses.clear()
        node.get_logger().info('Path cleared after navigation.')
        navigator.destroyNode()
    
    executor.shutdown()
    executor_thread.join()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
