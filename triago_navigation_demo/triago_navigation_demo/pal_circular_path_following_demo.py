#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from play_motion2_msgs.action import PlayMotion2
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
        self._action_client = ActionClient(self, PlayMotion2, '/play_motion2')
        self._is_successful = None
        self.subscription = self.create_subscription(
          Odometry,
          '/mobile_base_controller/odom',
          self.odom_callback,
          10)
        self.first_time = True
        self.start = None
        self.completed = False
        
    def odom_callback(self, msg):
        if self.first_time:
            self.start = PoseStamped()
            self.start.header.stamp = msg.header.stamp
            self.start.pose = msg.pose.pose
            self.first_time = False

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


    def navigation_routine(self):

        start = self.start
        self.get_logger().info('Odometry start point initialized.')
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        navigator = BasicNavigator()
        path = navigator.getPath(start, goal, 'FunctionPlanner', False)
        self.get_logger().info('Path received, following it.')

        navigator.followPath(path, 'TrackPose', 'general_goal_checker')
        navigator.waitUntilNav2Active()
        
        if navigator.getResult() == 'Succeeded':
            self.get_logger().info('Task succeeded')
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
         
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback.distance_to_goal < 1e-3:
                self.completed =  True
                break
            else:
                rclpy.spin_once(self)

        path.poses.clear()
        self.get_logger().info('Path cleared after navigation.')

        navigator.destroyNode()
            

def main(args=None):
    rclpy.init(args=args)
    node = CircularPathFollowingDemo()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    node._action_client.wait_for_server()

    node.send_goal('home', False)

    node.send_goal('torso_up', False)

    node.navigation_routine()
       
    node.send_goal('torso_down', False)
    node.completed = False
    node.navigation_routine()
    
    

    executor.shutdown()
    executor_thread.join()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
