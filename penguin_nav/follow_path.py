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
# X = "x"
# Y = "y"
# for pcl_pose
X = "/pcl_pose/pose/pose/position/x"
Y = "/pcl_pose/pose/pose/position/y"


class Controller:
    def __init__(self):
        self.navigator = BasicNavigator()

    def to_pose(self, x, y, yaw):
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
        return pose

    def go_through(self, poses):
        self.navigator.goThroughPoses([self.to_pose(x, y, yaw) for x, y, yaw in poses])

        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            if (feedback := self.navigator.getFeedback()) and i % 5 == 0:
                self.navigator.debug(
                    f"Estimated time of arrival: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.2f} seconds"
                )

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.navigator.info("Navigation succeeded")
        elif result == TaskResult.CANCELED:
            self.navigator.info("Navigation canceled")
        elif result == TaskResult.FAILED:
            self.navigator.info("Navigation failed")
        else:
            self.navigator.info("Navigation unknown result")

    def stop(self):
        self.navigator.info("Stopping navigation")
        self.navigator.cancelTask()


def df_to_poses(df: pd.DataFrame):
    poses = []

    for i in range(len(df)):
        poses.append((df[X][i], df[Y][i], 0))

    for i in range(len(poses) - 1):
        x1, y1, _ = poses[i]
        x2, y2, _ = poses[i + 1]
        yaw = math.atan2(y2 - y1, x2 - x1)
        poses[i] = (x1, y1, yaw)

    # repeat tail yaw
    poses[-1] = (poses[-1][0], poses[-1][1], poses[-2][2])
    return poses


def main():
    rclpy.init()

    controller = Controller()

    files = sys.argv[1:]

    try:
        for i, file in enumerate(files):
            controller.navigator.get_logger().info(
                f"Running {file} ... {i+1}/{len(files)}"
            )
            df = pd.read_csv(file)
            poses = df_to_poses(df)
            controller.go_through(poses)
            if len(files) > 1:
                val = input(
                    f"Finish {i+1}th file. Press enter to continue. Type 'q' to quit."
                )
                if val == "q":
                    break

    except KeyboardInterrupt:
        controller.stop()
        raise


if __name__ == "__main__":
    main()
