#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import rclpy.clock
from rclpy.duration import Duration
from scipy.spatial.transform import Rotation
import pandas as pd
import math
import sys

# Columns configuration
X="x"
Y="y"

class Controller:
    def __init__(self):
        self.navigator = BasicNavigator()


    def to_pose(self, x,y, yaw):
        q = Rotation.from_euler("z", yaw).as_quat()

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        print(x,y,yaw)
        return pose

    def go_through(self, poses):
        self.navigator.goThroughPoses([self.to_pose(x, y, yaw) for x, y, yaw in poses])

        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            if (feedback := self.navigator.getFeedback()) and i % 5 == 0:
                print(f"Estimated time of arrival: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.2f} seconds")

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Navigation succeeded")
        elif result == TaskResult.CANCELED:
            print("Navigation canceled")
        elif result == TaskResult.FAILED:
            print("Navigation failed")
        else:
            print("Navigation unknown result")


def main():
    rclpy.init()

    controller = Controller()


    df = pd.read_csv(sys.argv[1])

    poses = []

    for i in range(len(df)):
        poses.append((df[X][i], df[Y][i], 0))

    for i in range(len(poses) - 1):
        x1, y1, _ = poses[i]
        x2, y2, _ = poses[i+1]
        yaw = math.atan2(y2 - y1, x2 - x1)
        poses[i] = (x1, y1, yaw)

    # repeat tail yaw
    poses[-1] = (poses[-1][0], poses[-1][1], poses[-2][2])

    controller.go_through(poses)

    # go back to initial position
    # controller.go_through([(-2,-0.5,0)])

if __name__ == "__main__":
    main()
