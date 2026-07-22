#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseArray, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# Utils
def get_yaw(pose):
    q = pose.orientation
    _, _, yaw = euler_from_quaternion([q.x,q.y,q.z,q.w])
    return yaw #rad

def yaw_to_quaternion(yaw):
    return quaternion_from_euler(0.0, 0.0, yaw)

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def matrix2pose(T):
    tx = T[0][2]
    ty = T[1][2]
    theta = math.atan2(T[1][0], T[0][0])
    return tx, ty, theta

def transform_matrix(tx, ty, teta):
    c = math.cos(teta)
    s = math.sin(teta)
    return [
        [c, -s, tx],
        [s, c, ty],
        [0, 0, 1],
    ]

def inverse_transform(T):
    c = T[0][0]
    s = T[1][0]
    tx = T[0][2]
    ty = T[1][2]

    return [
        [c, s, -(c*tx+s*ty)],
        [-s, c, -(-s*tx+c*ty)],
        [0, 0, 1],
    ]

def matrix_multiplication(A,B):
    result = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                result[i][j] += A[i][k]*B[k][j]

    return result

class MapAlignment(Node):
    def __init__(self):
        super().__init__("map_alignment")

        self.robot_names = [
            "artbot1",
            "artbot2"
        ]

        # Entradas
        self.gt_poses = {}
        self.odom_poses = {}
        self.tf_broadcaster = TransformBroadcaster(self)

        for robot in self.robot_names:
            gt_topic = f"/{robot}/gt_pose_raw"
            odom_topic = f"/{robot}/odom"

            self.create_subscription(
                PoseArray,
                gt_topic,
                lambda msg, name=robot: self.gt_callback(msg, name),
                10
            )
            self.get_logger().info(f"Subscribed to {gt_topic}")

            self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, name=robot: self.odom_callback(msg, name),
                10
            )
            self.get_logger().info(f"Subscribed to {odom_topic}")

    def gt_callback(self, msg, robot):
        if not msg.poses:
            return
        self.gt_poses[robot] = msg.poses[0]
        self.publish_transforms()

    def odom_callback(self, msg, robot):
        self.odom_poses[robot] = msg.pose.pose
        self.publish_transforms()

    def publish_transforms(self):
        gt_poses_ok = len(self.gt_poses) == len(self.robot_names)
        odom_poses_ok = len(self.odom_poses) == len(self.robot_names)

        if (gt_poses_ok and odom_poses_ok):
            alignment_data = self.compute_alignment_data()
            transformations = self.compute_transformations(alignment_data)
            self.publish_reference_tf()

            for robot, T in transformations.items():
                self.publish_map_transform(robot, T)

    def publish_reference_tf(self):
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "merged_map"
        tf.child_frame_id = f"{self.robot_names[0]}_map"

        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)

    def publish_map_transform(self, robot, T):
        tf = TransformStamped()

        tx, ty, yaw = matrix2pose(T)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "merged_map"
        tf.child_frame_id = f"{robot}_map"

        tf.transform.translation.x = tx
        tf.transform.translation.y = ty
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)

    def relative_pose(self, pose_current, pose_other):
        xc = pose_current.position.x
        yc = pose_current.position.y
        xo = pose_other.position.x
        yo = pose_other.position.y
        yaw_c = get_yaw(pose_current)
        yaw_o = get_yaw(pose_other)

        dx = xo - xc
        dy = yo - yc
        c = math.cos(yaw_c)
        s = math.sin(yaw_c)

        x_rel = c * dx + s * dy
        y_rel = -s * dx + c * dy
        yaw_rel = normalize_angle(yaw_o - yaw_c)

        return {
            "translation": {
                "x": x_rel,
                "y": y_rel
            },
            "rotation": yaw_rel
        }

    def compute_alignment_data(self):
        alignment_data = {}

        for robot in self.robot_names:
            alignment_data[robot] = {}

        # Informações individuais (odometria)
        for robot in self.robot_names:
            pose = self.odom_poses[robot]

            alignment_data[robot][robot] = {
                "translation": {
                    "x": pose.position.x,
                    "y": pose.position.y
                },
                "rotation": get_yaw(pose)
            }

        # Informações relativas (ground truth)
        for robot_a in self.robot_names:
            for robot_b in self.robot_names:
                if robot_a == robot_b:
                    continue

                alignment_data[robot_a][robot_b] = self.relative_pose(
                    self.gt_poses[robot_a], self.gt_poses[robot_b]
                )
        return alignment_data
    
    def compute_transformations(self, alignment_data):
        transformations = {}
        reference_robot = self.robot_names[0]

        for robot in self.robot_names:
            if robot == reference_robot:
                continue
        
            current = alignment_data[reference_robot][reference_robot]
            relative = alignment_data[reference_robot][robot]
            other = alignment_data[robot][robot]

            Tc = transform_matrix(
                current["translation"]["x"],
                current["translation"]["y"],
                current["rotation"]
            )
            Tr = transform_matrix(
                relative["translation"]["x"],
                relative["translation"]["y"],
                relative["rotation"]
            )
            To = transform_matrix(
                other["translation"]["x"],
                other["translation"]["y"],
                other["rotation"]
            )
            T = matrix_multiplication(
                matrix_multiplication(Tc, Tr),
                inverse_transform(To)
            )

            transformations[robot] = T
    
        return transformations

def main(args=None):

    rclpy.init(args=args)

    node = MapAlignment()

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