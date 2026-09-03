#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError

from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class SlamPosePub(Node):
    def __init__(self):
        super().__init__("slam_pose_pub")

        self.declare_parameter(
            'robot_names',
            ['artbot1', 'artbot2']
        )
        self.robot_names = self.get_parameter(
            'robot_names'
        ).get_parameter_value().string_array_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_publishers = {}
        for robot in self.robot_names:
            slam_pose_topic = f'/{robot}/slam_pose'

            self.pose_publishers[robot] = self.create_publisher(
                PoseStamped,
                slam_pose_topic,
                10
            )
            self.get_logger().info(f"Publishing to {slam_pose_topic}")

        self.timer = self.create_timer(0.5, self.publish_pose)

    def publish_pose(self):
        for robot in self.robot_names:
            frame_id = f'{robot}_map'
            child_frame_id = f'{robot}_base_link'

            try:
                tf = self.tf_buffer.lookup_transform(frame_id, child_frame_id, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warn(f'Could not transform {frame_id} to {child_frame_id}: {ex}')
                continue

            pose = PoseStamped()
            pose.header.stamp = tf.header.stamp
            pose.header.frame_id = tf.header.frame_id
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation

            try:
                self.pose_publishers[robot].publish(pose)
            except RCLError:
                return

def main(args=None):
    rclpy.init(args=args)

    node = SlamPosePub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()