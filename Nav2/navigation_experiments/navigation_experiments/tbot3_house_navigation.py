"""
Sets the initial pose of the turtlebot on the world map and navigates to a point.

How to run:
    1) Start TB3 in Gazebo: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 

    2) Start nav2 navigation and pass in the YAML file of a generated map:
        ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=ROS_Resources/maps/my_map.yaml

        Note: The path to the directory file and the map file must exist or this will fail.

    3) Run this node: ros2 run navigation_experiments navigate_to_point 

"""



#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


class SetInitialPoseNode(Node):
    def __init__(self):
        super().__init__("tbot3_house_navigation")

        self.nav = BasicNavigator()

        # Run once, shortly after spin starts
        self._timer = self.create_timer(0.1, self._do_once)

    def create_pose_stamped(self, navigator : BasicNavigator, position_x, position_y, orientation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def _do_once(self):
        # Stop the timer so it doesn't fire again
        self._timer.cancel()

        # --- set initial pose
        initial_pose = self.create_pose_stamped(self.nav, 0.0, 0.0, 0.0)
        # q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = "map"
        # initial_pose.header.stamp = self.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 0.0
        # initial_pose.pose.position.y = 0.0
        # initial_pose.pose.position.z = 0.0
        # initial_pose.pose.orientation.x = q_x
        # initial_pose.pose.orientation.y = q_y
        # initial_pose.pose.orientation.z = q_z
        # initial_pose.pose.orientation.w = q_w
        # self.nav.setInitialPose(initial_pose)

        # --- wait for Nav2
        self.nav.waitUntilNav2Active()

        # -- Send Nav2 goal
        # PI == 3.14 == 180
        # PI/2 == 1.57 == 90
        goal_pose1 = self.create_pose_stamped(self.nav, -3.5, 3.5, -3.14)
        goal_pose2 = self.create_pose_stamped(self.nav, -4.6, 0.0, -1.57)
        goal_pose3 = self.create_pose_stamped(self.nav, -3.9, -2.66, -3.14)
        # q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 1.57)
        # goal_pose = PoseStamped()
        # goal_pose.header.frame_id = 'map'
        # goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        # goal_pose.pose.position.x = 3.5
        # goal_pose.pose.position.y = 1.0
        # goal_pose.pose.position.z = 0.0
        # goal_pose.pose.orientation.x = q_x
        # goal_pose.pose.orientation.y = q_y
        # goal_pose.pose.orientation.z = q_z
        # goal_pose.pose.orientation.w = q_w

        # -- got to pose 1
        # self.nav.goToPose(goal_pose1)
        # while not self.nav.isTaskComplete():
        #     feedback = self.nav.getFeedback()
        #     # print(feedback)

        # -- Follow waypoints
        waypoints = [goal_pose1, goal_pose2, goal_pose3]
        self.nav.followWaypoints(waypoints)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()

        
        print(self.nav.getResult())


        # self.get_logger().info("Initial pose set; Nav2 active; shutting down cleanly.")

        # Now it's safe to shutdown because spin is running and executor exists
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SetInitialPoseNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
