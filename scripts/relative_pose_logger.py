#!/usr/bin/env python3

import math
import json

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class RelativePoseLogger(Node):

    def __init__(self):
        super().__init__("relative_pose_logger")

        self.robot_names = [
            "artbot1",
            "artbot2"
        ]

        # Ground truth (PoseArray)
        self.gt_poses = {}

        # Odometria
        self.odom_poses = {}

        self.finished = False

        self.done_future = Future()

        for robot in self.robot_names:

            gt_topic = f"/{robot}/gt_pose_raw"
            odom_topic = f"/{robot}/odom"

            self.create_subscription(
                PoseArray,
                gt_topic,
                lambda msg, name=robot: self.gt_callback(msg, name),
                10
            )

            self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, name=robot: self.odom_callback(msg, name),
                10
            )

            self.get_logger().info(f"Subscribed to {gt_topic}")
            self.get_logger().info(f"Subscribed to {odom_topic}")

        self.declare_parameter(
            name="output_file",
            value="relative_poses.json"
        )

        self.output_file = self.get_parameter(
            name="output_file"
        ).value

    def gt_callback(self, msg, robot):

        if self.finished:
            return

        if not msg.poses:
            return

        self.gt_poses[robot] = msg.poses[0]

        self.check_ready()

    def odom_callback(self, msg, robot):

        if self.finished:
            return

        self.odom_poses[robot] = msg.pose.pose

        self.check_ready()

    def check_ready(self):

        if (
            len(self.gt_poses) == len(self.robot_names)
            and
            len(self.odom_poses) == len(self.robot_names)
        ):

            self.compute_relative_poses()

    def get_yaw(self, pose):

        q = pose.orientation

        _, _, yaw = euler_from_quaternion([
            q.x,
            q.y,
            q.z,
            q.w
        ])

        return yaw #rad

    def normalize_angle(self, angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def relative_pose(self, pose_current, pose_other):

        xc = pose_current.position.x
        yc = pose_current.position.y

        xo = pose_other.position.x
        yo = pose_other.position.y

        yaw_c = self.get_yaw(pose_current)
        yaw_o = self.get_yaw(pose_other)

        dx = xo - xc
        dy = yo - yc

        c = math.cos(yaw_c)
        s = math.sin(yaw_c)

        x_rel = c * dx + s * dy
        y_rel = -s * dx + c * dy

        yaw_rel = self.normalize_angle(
            yaw_o - yaw_c
        )

        return {
            "translation": {
                "x": x_rel,
                "y": y_rel
            },
            "rotation": yaw_rel
        }

    def compute_relative_poses(self):

        self.finished = True

        data = {}

        for robot in self.robot_names:
            data[robot] = {}

        # Informações individuais (odometria)

        for robot in self.robot_names:

            pose = self.odom_poses[robot]

            data[robot][robot] = {

                "translation": {
                    "x": pose.position.x,
                    "y": pose.position.y
                },
                "rotation": self.get_yaw(pose)
            }

        # Informações relativas (ground truth)

        for robot_a in self.robot_names:

            for robot_b in self.robot_names:

                if robot_a == robot_b:
                    continue

                data[robot_a][robot_b] = self.relative_pose(
                    self.gt_poses[robot_a],
                    self.gt_poses[robot_b]
                )

        with open(self.output_file, "w") as file:

            json.dump(
                data,
                file,
                indent=4
            )

        self.get_logger().info(
            "Arquivo relative_poses.json salvo."
        )

        self.done_future.set_result(True)

def main(args=None):

    rclpy.init(args=args)

    node = RelativePoseLogger()

    try:

        rclpy.spin_until_future_complete(
            node,
            node.done_future
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()