#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rclpy.action import ActionClient
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
