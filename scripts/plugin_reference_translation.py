#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseArray, TransformStamped, Quaternion
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

def quaternion_to_yaw(pose):
    q = pose.orientation
    _, _, yaw = euler_from_quaternion([q.x,q.y,q.z,q.w])
    return yaw #rad

def yaw_to_quaternion(yaw):
    q = Quaternion()
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    q.x, q.y, q.z, q.w = qx, qy, qz, qw
    return q

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class PluginReferenceTranslation(Node):
    def __init__(self):
        super().__init__('reference_translation')

        self.declare_parameter(
            'robot_names',
            ['artbot1', 'artbot2']
        )
        robot_names = self.get_parameter(
            'robot_names'
        ).get_parameter_value().string_array_value

        self.robots = {}
        self.odom_publishers = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        
        for robot in robot_names:
            raw_odom_topic = f'/{robot}/odom_raw'
            raw_pose_topic = f'/{robot}/pose_raw'
            odom_topic = f'/{robot}/odom'

            self.robots[robot] = {
                'initialized': False,
                'x0': 0.0,
                'y0': 0.0,
                'yaw0': 0.0
            }

            self.create_subscription(
                Odometry,
                raw_odom_topic,
                lambda msg, r=robot: self.odom_callback(msg, r),
                10
            )
            self.get_logger().info(f"Subscribed to {raw_odom_topic}")

            self.create_subscription(
                PoseArray,
                raw_pose_topic,
                lambda msg, r=robot: self.pose_callback(msg, r),
                10
            )
            self.get_logger().info(f"Subscribed to {raw_pose_topic}")

            self.odom_publishers[robot] = self.create_publisher(
                Odometry,
                odom_topic,
                10
            )
            self.get_logger().info(f"Publishing to {odom_topic}")

    def pose_callback(self, msg, robot):
        if len(msg.poses) == 0:
            return
    
        if not self.robots[robot]['initialized']:
            self.robots[robot]['x0'] = msg.poses[0].position.x
            self.robots[robot]['y0'] = msg.poses[0].position.y
            self.robots[robot]['yaw0'] = quaternion_to_yaw(msg.poses[0])
            self.robots[robot]['initialized'] = True

            self.get_logger().info(
                f"[{robot}] Initial pose captured "
                f"({self.robots[robot]['x0']:.3f}, {self.robots[robot]['y0']:.3f}, {self.robots[robot]['yaw0']:.3f})"
            )
            return

        tf = self.compose_tf(msg, robot)
        self.tf_broadcaster.sendTransform(tf)

    def odom_callback(self, msg, robot):
        if not self.robots[robot]['initialized']:
            return
    
        new_odom = self.new_odometry(msg, robot)
        self.odom_publishers[robot].publish(new_odom)

    def translate_pose(self, pose, robot):
        x = pose.position.x
        y = pose.position.y
        yaw = quaternion_to_yaw(pose)

        tx = x - self.robots[robot]['x0']
        ty = y - self.robots[robot]['y0']
        yaw0 = self.robots[robot]['yaw0']

        new_x = math.cos(yaw0)*tx + math.sin(yaw0)*ty
        new_y = - math.sin(yaw0)*tx + math.cos(yaw0)*ty
        new_yaw = normalize_angle(yaw - yaw0)

        return new_x, new_y, new_yaw

    def new_odometry(self, msg, robot):
        pose = msg.pose.pose
        new_x, new_y, new_yaw = self.translate_pose(pose, robot)

        new_odom = Odometry()

        new_odom.header.stamp = msg.header.stamp
        new_odom.header.frame_id = msg.header.frame_id
        new_odom.child_frame_id = msg.child_frame_id
        
        new_odom.pose.pose.position.x = new_x
        new_odom.pose.pose.position.y = new_y
        new_odom.pose.pose.orientation = yaw_to_quaternion(new_yaw)
        new_odom.pose.covariance = msg.pose.covariance

        new_odom.twist = msg.twist

        return new_odom

    def compose_tf(self, msg, robot):
        pose = msg.poses[0]
        new_x, new_y, new_yaw = self.translate_pose(pose, robot)

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = f'{robot}_odom'
        tf.child_frame_id = f'{robot}_base_link'

        tf.transform.translation.x = new_x
        tf.transform.translation.y = new_y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = yaw_to_quaternion(new_yaw)

        return tf

def main(args=None):
    rclpy.init(args=args)
    node = PluginReferenceTranslation()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()