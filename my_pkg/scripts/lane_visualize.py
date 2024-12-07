#!/usr/bin/env python
import rospy
import numpy as np
import os
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

"""
Constant Definition
"""
WIDTH = 0.2032  # (m)
WHEEL_LENGTH = 0.0381  # (m)
MAX_STEER = 0.36  # (rad)


class LaneVisualize:
    """
    Class for lane visualization
    """

    def __init__(self):
        # Initialize the node
        rospy.init_node("lane_visualize_node")

        # ROS Params
        self.real_test = rospy.get_param("~real_test", False)
        self.map_name = rospy.get_param("~map_name", "final1")
        self.num_lanes = rospy.get_param("~num_lanes", 1)
        self.lane_files = rospy.get_param("~lane_files", ["centerline"])
        self.traj_file = rospy.get_param("~traj_file", "traj_race_cl")

        # Optimal Trajectory
        traj_csv_loc = os.path.join(
            "/home/patrick/trajectory_generator/f1tenth-racing-stack-ICRA22/trajectory_generator/outputs/",
            self.map_name,
            self.traj_file + ".csv",
        )
        traj_data = np.loadtxt(traj_csv_loc, delimiter=";", skiprows=0)
        self.num_traj_pts = len(traj_data)
        self.traj_x = traj_data[:, 1]
        self.traj_y = traj_data[:, 2]
        self.traj_pos = np.vstack((self.traj_x, self.traj_y))
        self.traj_yaw = traj_data[:, 3]
        self.traj_v = traj_data[:, 5]
        self.v_min = np.min(self.traj_v)
        self.v_max = np.max(self.traj_v)

        # Lanes Waypoints
        self.num_lane_pts = []
        self.lane_x = []
        self.lane_y = []

        assert len(self.lane_files) == self.num_lanes
        for i in range(self.num_lanes):
            lane_csv_loc = os.path.join(
                "/home/patrick/trajectory_generator/f1tenth-racing-stack-ICRA22/trajectory_generator/outputs/",
                self.map_name,
                self.lane_files[i] + ".csv",
            )
            lane_data = np.loadtxt(lane_csv_loc, delimiter=",")
            self.num_lane_pts.append(len(lane_data))
            self.lane_x.append(lane_data[:, 0])
            self.lane_y.append(lane_data[:, 1])

        # Topics & Publishers
        self.traj_pub_ = rospy.Publisher(
            "/global_path/optimal_trajectory", Marker, queue_size=10
        )

        self.lane_pub_ = []
        for i in range(self.num_lanes):
            lane_topic = "/global_path/lane_" + str(i)
            self.lane_pub_.append(rospy.Publisher(lane_topic, Marker, queue_size=10))

        # Timer for visualization
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)

    def timer_callback(self, event):
        visualize = rospy.get_param("~visualize", True)
        if visualize:
            self.visualize_global_path()
            self.visualize_lanes()

    def visualize_global_path(self):
        # Publish trajectory waypoints
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 0
        marker.ns = "global_planner"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = []
        marker.colors = []

        for i in range(self.num_traj_pts + 1):
            this_point = Point()
            this_point.x = self.traj_x[i % self.num_traj_pts]
            this_point.y = self.traj_y[i % self.num_traj_pts]
            marker.points.append(this_point)

            this_color = ColorRGBA()
            speed_ratio = (self.traj_v[i % self.num_traj_pts] - self.v_min) / (
                self.v_max - self.v_min
            )
            this_color.a = 1.0
            this_color.r = 1 - speed_ratio
            this_color.g = speed_ratio
            marker.colors.append(this_color)

        marker.scale.x = 0.04
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.orientation.w = 1.0

        self.traj_pub_.publish(marker)

    def visualize_lanes(self):
        # Publish lane waypoints
        for lane_idx in range(self.num_lanes):
            num_pts = self.num_lane_pts[lane_idx]
            target_x = self.lane_x[lane_idx]
            target_y = self.lane_y[lane_idx]

            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = 0
            marker.ns = "global_planner"
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.points = []
            marker.colors = []
            for i in range(num_pts + 1):
                this_point = Point()
                this_point.x = target_x[i % num_pts]
                this_point.y = target_y[i % num_pts]
                marker.points.append(this_point)

                this_color = ColorRGBA()
                this_color.a = 1.0
                this_color.r = 0.5
                this_color.g = 0.5
                this_color.b = 0.5
                marker.colors.append(this_color)

            marker.scale.x = 0.05
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.pose.orientation.w = 1.0

            self.lane_pub_[lane_idx].publish(marker)


if __name__ == "__main__":
    print("Lane Visualize Initialized")
    LaneVisualize()
    rospy.spin()
