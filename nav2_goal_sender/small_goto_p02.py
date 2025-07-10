#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # Import Odometry message type
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# Define your desired goal coordinates and orientation (in degrees)
goal_x = 2.11
goal_y = 2.55
# Format for yaw
# quaternion [x, y, z, w]
# degree [deg]
goal_yaw = [0.0, 0.0, 0.57, 0.82]
quaternion = True

class SimpleGoalNavigator(Node):

    def __init__(self):
        super().__init__('simple_goal_navigator')
        self.navigator = BasicNavigator()

        self.current_robot_pose_odom = None
        self.pose_received = False

        # Create a subscriber to the /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,                  # Type of message /odom publishes
            '/odom',                   # Topic name
            self.odom_callback,        # Callback function
            10                         # QoS history depth
        )
        self.get_logger().info('Subscribing to /odom to get current robot position...')

        # Wait until a valid initial pose is received from Odometry
        self.get_logger().info('Waiting for Odometry pose to be received...')
        while not self.pose_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not rclpy.ok():
            self.get_logger().error("ROS2 context shut down before initial pose was received.")
            return

        self.get_logger().info('Odometry pose received! Setting initial pose for BasicNavigator.')
        
        # Convert Odometry message's pose to PoseStamped for setInitialPose
        initial_pose_stamped = PoseStamped()
        initial_pose_stamped.header = self.current_robot_pose_odom.header
        initial_pose_stamped.pose = self.current_robot_pose_odom.pose.pose

        self.navigator.setInitialPose(initial_pose_stamped)

        # Wait for Nav2 to become active
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active! Sending goal...')

    def odom_callback(self, msg):
        # Callback function to store the latest robot pose from odometry.
        self.current_robot_pose_odom = msg
        self.pose_received = True
        self.get_logger().info(f"Received odom pose: X={msg.pose.pose.position.x:.2f}, Y={msg.pose.pose.position.y:.2f}")

    def navigate_to_goal(self, x, y, yaw, is_quaternion = False):
        
        if not self.pose_received:
            self.get_logger().error("Cannot navigate: Initial pose not received from odometry yet.")
            return
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)

        yaw_degree = 0.0;

        if(is_quaternion):
            yaw_degree = math.degrees(euler_from_quaternion([yaw[0], yaw[1], yaw[2], yaw[3]])[2])
            goal_pose.pose.orientation.x = yaw[0]
            goal_pose.pose.orientation.y = yaw[1]
            goal_pose.pose.orientation.z = yaw[2]
            goal_pose.pose.orientation.w = yaw[3]
        else:
            # Convert yaw from degrees to radians and then to quaternion
            yaw_radians = math.radians(yaw[0])
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_radians)
            goal_pose.pose.orientation.x = qx
            goal_pose.pose.orientation.y = qy
            goal_pose.pose.orientation.z = qz
            goal_pose.pose.orientation.w = qw

        self.get_logger().info(f'Attempting to navigate to X:{x}, Y:{y}, Yaw:{yaw_degree} degrees.')
        self.navigator.goToPose(goal_pose)

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(f'Estimated time remaining: {feedback.estimated_time_remaining:.2f} seconds. Current speed: {feedback.current_speed:.2f} m/s')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Goal failed!')
        else:
            self.get_logger().info('Goal has unknown result.')

def main(args=None):
    rclpy.init(args=args)
    navigator_node = SimpleGoalNavigator()

    # Wait user
    input("Press enter to start navigation!")

    navigator_node.navigate_to_goal(goal_x, goal_y, goal_yaw, quaternion)

    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()