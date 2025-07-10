#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler

# Define your desired goal coordinates and orientation (in degrees)
goal_x = 5.0
goal_y = -3.0
goal_yaw = 90.0 # Robot will face 90 degrees (east) at the goal

class SimpleGoalNavigator(Node):

    def __init__(self):
        super().__init__('simple_goal_navigator')
        self.navigator = BasicNavigator()

        # Set the initial pose of the robot if not already localized (optional)
        # This is useful if your robot starts at an unknown location.
        # Otherwise, if Nav2 is already running and localized, you can skip this.
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0  # Set your robot's initial X position
        initial_pose.pose.position.y = 0.0  # Set your robot's initial Y position
        initial_pose.pose.orientation.w = 1.0 # No rotation
        self.navigator.setInitialPose(initial_pose)

        # Wait for Nav2 to become active
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active! Sending goal...')

    def navigate_to_goal(self, x, y, yaw_degrees):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)

        # Convert yaw from degrees to radians and then to quaternion
        yaw_radians = math.radians(yaw_degrees)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_radians)
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        self.get_logger().info(f'Attempting to navigate to X:{x}, Y:{y}, Yaw:{yaw_degrees} degrees.')
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

    navigator_node.navigate_to_goal(goal_x, goal_y, goal_yaw)

    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()