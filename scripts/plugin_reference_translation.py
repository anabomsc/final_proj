#!/usr/bin/env python3

import math
import numpy as np

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

def input_noise(gaussian_stddev):
    noise = np.random.normal(0.0, gaussian_stddev)
    return noise

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

        self.declare_parameter(
            name="gaussian_noise",
            value=[0.0, 0.0]
        )
        self.gaussian_noise = self.get_parameter(
            name="gaussian_noise"
        ).value

        self.robots = {}
        self.odom_publishers = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        
        for robot in robot_names:
            raw_odom_topic = f'/{robot}/odom_raw'
            odom_topic = f'/{robot}/odom'

            self.robots[robot] = {
                'initialized': False,
                'previous_time': 0.0,
                'odom_x': 0.0,
                'odom_y': 0.0,
                'odom_yaw': 0.0
            }

            self.create_subscription(
                Odometry,
                raw_odom_topic,
                lambda msg, r=robot: self.odom_callback(msg, r),
                10
            )
            self.get_logger().info(f"Subscribed to {raw_odom_topic}")

            self.odom_publishers[robot] = self.create_publisher(
                Odometry,
                odom_topic,
                10
            )
            self.get_logger().info(f"Publishing to {odom_topic}")

    def odom_callback(self, msg, robot):
        state = self.robots[robot]
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not state['initialized']:
            state['previous_time'] = current_time
            state['odom_x'] = 0.0
            state['odom_y'] = 0.0
            state['odom_yaw'] = 0.0
            state['initialized'] = True

            return

        dt = current_time - state['previous_time']
        if dt <= 0.0: return

        v = msg.twist.twist.linear.x + input_noise(self.gaussian_noise[0])
        omega = msg.twist.twist.angular.z + input_noise(self.gaussian_noise[1])
        theta = state['odom_yaw']

        state['odom_x'] += v * math.cos(theta) * dt
        state['odom_y'] += v * math.sin(theta) * dt
        state['odom_yaw'] = normalize_angle(theta + omega * dt)
        state['previous_time'] = current_time

        new_pose = [state['odom_x'], state['odom_y'], state['odom_yaw']]

        new_odom = self.new_odometry(new_pose, msg)
        tf = self.compose_tf(new_pose, msg)
        self.odom_publishers[robot].publish(new_odom)
        self.tf_broadcaster.sendTransform(tf)

    def new_odometry(self, position, msg):
        new_odom = Odometry()
        new_odom.header.stamp = msg.header.stamp
        new_odom.header.frame_id = msg.header.frame_id
        new_odom.child_frame_id = msg.child_frame_id

        new_odom.pose.pose.position.x = position[0]
        new_odom.pose.pose.position.y = position[1]
        new_odom.pose.pose.orientation = yaw_to_quaternion(position[2])
        new_odom.pose.covariance = msg.pose.covariance

        new_odom.twist = msg.twist

        return new_odom

    def compose_tf(self, position, msg):
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = msg.header.frame_id
        tf.child_frame_id = msg.child_frame_id

        tf.transform.translation.x = position[0]
        tf.transform.translation.y = position[1]
        tf.transform.translation.z = 0.0
        tf.transform.rotation = yaw_to_quaternion(position[2])

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