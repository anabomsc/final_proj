#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RelativeGroundTruth(Node):

    def __init__(self):

        super().__init__('gt_relative_odom')

        self.declare_parameter(
            'robot_names',
            ['artbot1', 'artbot2']
        )

        robot_names = self.get_parameter(
            'robot_names'
        ).get_parameter_value().string_array_value

        self.robots = {}

        for robot in robot_names:

            self.robots[robot] = {
                'initialized': False,
                'x0': 0.0,
                'y0': 0.0,
                'yaw0': 0.0
            }

            self.create_subscription(
                PoseArray,
                f'/{robot}/gt_pose_raw',
                lambda msg, r=robot: self.pose_callback(msg, r),
                10
            )

            self.robots[robot]['publisher'] = self.create_publisher(
                Odometry,
                f'/{robot}/gt_pose_rel',
                10
            )

            self.get_logger().info(
                f'Configured robot: {robot}'
            )

    def pose_callback(self, msg, robot):

        if len(msg.poses) == 0:
            return

        pose = msg.poses[0]

        x = pose.position.x
        y = pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)

        robot_data = self.robots[robot]

        if not robot_data['initialized']:

            robot_data['x0'] = x
            robot_data['y0'] = y
            robot_data['yaw0'] = yaw
            robot_data['initialized'] = True

            self.get_logger().info(
                f'[{robot}] Initial pose captured '
                f'({x:.3f}, {y:.3f}, {yaw:.3f})'
            )

            return

        dx = x - robot_data['x0']
        dy = y - robot_data['y0']

        yaw0 = robot_data['yaw0']

        #
        # Transformação world -> frame inicial
        #
        x_rel = (
            math.cos(yaw0) * dx +
            math.sin(yaw0) * dy
        )

        y_rel = (
            -math.sin(yaw0) * dx +
            math.cos(yaw0) * dy
        )

        yaw_rel = normalize_angle(
            yaw - yaw0
        )

        odom = Odometry()

        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = f'{robot}_gt_odom'

        odom.child_frame_id = f'{robot}_base_link'

        odom.pose.pose.position.x = x_rel
        odom.pose.pose.position.y = y_rel
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = quaternion_from_yaw(
            yaw_rel
        )

        robot_data['publisher'].publish(odom)


def main(args=None):

    rclpy.init(args=args)

    node = RelativeGroundTruth()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()