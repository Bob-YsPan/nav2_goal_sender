#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
import math
from rclpy.duration import Duration # For checking feedback time

class WaypointFollowerNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_follower_navigator')
        self.navigator = BasicNavigator()

        # Set the initial pose if not already localized
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active! Starting waypoint following...')

    def follow_waypoints(self, waypoint_list):
        # Convert yaw from degrees to radians and then to quaternion for each waypoint
        waypoints = []
        for x, y, yaw_degrees in waypoint_list:
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
            waypoints.append(pose)

        self.get_logger().info(f'Attempting to follow {len(waypoints)} waypoints.')
        self.navigator.followWaypoints(waypoints)

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(waypoints)}')
                self.get_logger().info(f'Estimated time remaining: {feedback.estimated_time_remaining:.2f} seconds. Current speed: {feedback.current_speed:.2f} m/s')
                
                # Example of adding a custom action at a waypoint
                # The Nav2 Waypoint Follower can be configured with plugins
                # to execute specific tasks at each waypoint (e.g., "wait_at_waypoint")
                # This feedback check is just for monitoring progress.
                # if feedback.current_waypoint == 0 and feedback.navigation_duration > Duration(seconds=10).nanoseconds / 1e9:
                #     self.get_logger().info('Robot has been at first waypoint for 10 seconds, cancelling for demo.')
                #     self.navigator.cancelTask()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Waypoint following succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Waypoint following was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Waypoint following failed!')
        else:
            self.get_logger().info('Waypoint following has unknown result.')

def main(args=None):
    rclpy.init(args=args)
    navigator_node = WaypointFollowerNavigator()

    # Wait user
    input("Press enter to start navigation!")

    # Define a list of waypoints (x, y, yaw_degrees)
    # The robot will stop and "achieve" each waypoint before moving to the next.
    waypoints_for_patrol = [
        (2.0, 0.0, 0.0),    # Waypoint 1
        (2.0, 2.0, 90.0),   # Waypoint 2
        (0.0, 2.0, 180.0),  # Waypoint 3
        (0.0, 0.0, -90.0)   # Waypoint 4 (back to start, facing south)
    ]

    navigator_node.follow_waypoints(waypoints_for_patrol)

    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()