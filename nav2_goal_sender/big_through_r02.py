#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
import math

class ThroughPosesNavigator(Node):

    def __init__(self):
        super().__init__('through_poses_navigator')
        self.navigator = BasicNavigator()

        # Set the initial pose if not already localized
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 0.0
        # initial_pose.pose.position.y = 0.0
        # initial_pose.pose.orientation.w = 1.0
        # self.navigator.setInitialPose(initial_pose)

        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active! Sending poses...')

    def navigate_through_poses(self, pose_list):
        # Convert yaw from degrees to radians and then to quaternion for each pose
        goal_poses = []
        for x, y, yaw_degrees in pose_list:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            
            yaw_radians = math.radians(yaw_degrees)
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_radians)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            goal_poses.append(pose)

        self.get_logger().info(f'Attempting to navigate through {len(goal_poses)} poses.')
        self.navigator.goThroughPoses(goal_poses)

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(f'Estimated time remaining: {feedback.estimated_time_remaining:.2f} seconds. Current speed: {feedback.current_speed:.2f} m/s')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation through poses succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Navigation through poses was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation through poses failed!')
        else:
            self.get_logger().info('Navigation through poses has unknown result.')

def main(args=None):
    rclpy.init(args=args)
    navigator_node = ThroughPosesNavigator()

    # Wait user
    input("Press enter to start navigation!")

    # Define a list of poses (x, y, yaw_degrees)
    # The robot will try to navigate through these in order.
    # It will not necessarily stop at each intermediate point.
    poses_to_visit = [
        (2.0, 1.0, 0.0),    # First intermediate point, facing 0 degrees
        (4.0, 3.0, 90.0),   # Second intermediate point, facing 90 degrees
        (1.0, 5.0, 180.0)   # Final goal point, facing 180 degrees
    ]

    navigator_node.navigate_through_poses(poses_to_visit)

    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()