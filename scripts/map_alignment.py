#!/usr/bin/env python3

import math
import numpy as np
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseArray, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy._rclpy_pybind11 import RCLError

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

def input_noise(gaussian_mean, gaussian_stddev, Tr):
    noise = np.random.normal(gaussian_mean, gaussian_stddev, np.shape(Tr))
    noise[2] = [0.0, 0.0, 0.0]
    return noise

class MapAlignment(Node):
    def __init__(self):
        super().__init__("map_alignment")

        self.declare_parameter("alignment_output_file", "")
        self.alignment_output_file = (
            self.get_parameter("alignment_output_file")
            .get_parameter_value()
            .string_value
        )

        self.declare_parameter(
            name="gaussian_noise",
            value=[0.0, 0.0]
        )
        self.gaussian_noise = self.get_parameter(
            name="gaussian_noise"
        ).value

        self.robot_names = [
            "artbot1",
            "artbot2"
        ]

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Entradas
        self.gt_poses = {}
        self.odom_poses = {}
        self.local_maps = {}
        self.map_publishers = {}
        self.tf_broadcaster = TransformBroadcaster(self)

        for robot in self.robot_names:
            gt_topic = f"/{robot}/gt_pose_raw"
            odom_topic = f"/{robot}/odom"
            local_map_topic = f"/{robot}/map"
            aligned_map_topic = f"/{robot}/map_aligned"

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

            self.create_subscription(
                OccupancyGrid,
                local_map_topic,
                lambda msg, name=robot: self.map_callback(msg, name),
                10
            )
            self.get_logger().info(f"Subscribed to {local_map_topic}")

            self.map_publishers[robot] = self.create_publisher(
                OccupancyGrid,
                aligned_map_topic,
                map_qos,
            )
            self.get_logger().info(f"Publishing to {aligned_map_topic}")

    def save_alignment_data(self, alignment_data, transformation_data, noise_data):
        if not self.alignment_output_file:
            return
        try:
            directory = os.path.dirname(self.alignment_output_file)
            if directory:
                os.makedirs(directory, exist_ok=True)

            with open(self.alignment_output_file, "w") as f1:
                json.dump(alignment_data, f1, indent=4)

            serializable_info = {
                "transformation": {},
                "noise": {},
            }
            for robot, T in transformation_data.items():
                if hasattr(T, "tolist"):
                    serializable_info["transformation"][robot] = T.tolist()
                else:
                    serializable_info["transformation"][robot] = T

            for robot, noise in noise_data.items():
                if hasattr(noise, "tolist"):
                    serializable_info["noise"][robot] = noise.tolist()
                else:
                    serializable_info["noise"][robot] = noise
            
            transformation_output_file = os.path.join(directory, "transformation_data.json")
            with open(transformation_output_file, "w") as f2:
                json.dump(serializable_info, f2, indent=4)
        except Exception as e:
            self.get_logger().error(
                f"Failed to save alignment or transformation data: {e}"
            )

    def gt_callback(self, msg, robot):
        if not msg.poses:
            return
        self.gt_poses[robot] = msg.poses[0]
        self.publish_transforms()

    def odom_callback(self, msg, robot):
        self.odom_poses[robot] = msg.pose.pose
        self.publish_transforms()

    def map_callback(self, msg, robot):
        self.local_maps[robot] = msg
        self.publish_transforms()

    def publish_transforms(self):
        gt_poses_ok = len(self.gt_poses) == len(self.robot_names)
        odom_poses_ok = len(self.odom_poses) == len(self.robot_names)
        local_maps_ok = len(self.local_maps) == len(self.robot_names)

        if (gt_poses_ok and odom_poses_ok and local_maps_ok):
            alignment_data = self.compute_alignment_data()
            transformations, noises = self.compute_transformations(alignment_data)
            self.publish_reference_tf(self.local_maps[self.robot_names[0]])

            for robot, T in transformations.items():
                self.publish_map_transform(robot, T)
                aligned = self.map_alignment(
                    self.local_maps[robot], T
                )

                try:
                    self.map_publishers[robot].publish(aligned)
                    self.save_alignment_data(alignment_data, transformations, noises)
                except RCLError:
                    return

    def publish_reference_tf(self, map_ref):
        tf = TransformStamped()
        aligned = OccupancyGrid()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "merged_map"
        tf.child_frame_id = f"{self.robot_names[0]}_map"

        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0

        aligned = map_ref
        aligned.header.frame_id = "merged_map"

        try:
            self.tf_broadcaster.sendTransform(tf)
            self.map_publishers[self.robot_names[0]].publish(aligned)
        except RCLError:
            return

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

        try:
            self.tf_broadcaster.sendTransform(tf)
        except RCLError:
            return

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
        noises = {}
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
            gaussian_mean = self.gaussian_noise[0]
            gaussian_stddev = self.gaussian_noise[1]
            noise = input_noise(gaussian_mean, gaussian_stddev, Tr)
            Tr = Tr + noise
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
            noises[robot] = noise
    
        return transformations, noises

    def map_alignment(self, grid_robot, T):
        aligned = OccupancyGrid()

        aligned.header.stamp = grid_robot.header.stamp
        aligned.header.frame_id = "merged_map"

        aligned.info.map_load_time = grid_robot.info.map_load_time
        aligned.info.resolution = grid_robot.info.resolution
        aligned.info.width = grid_robot.info.width
        aligned.info.height = grid_robot.info.height
        
        origin = grid_robot.info.origin
        origin_T = transform_matrix(
            origin.position.x, origin.position.y, get_yaw(origin)
        )
        new_origin = matrix_multiplication(T, origin_T)
        tx, ty, yaw = matrix2pose(new_origin)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        aligned.info.origin.position.x = tx
        aligned.info.origin.position.y = ty
        aligned.info.origin.orientation.x = qx
        aligned.info.origin.orientation.y = qy
        aligned.info.origin.orientation.z = qz
        aligned.info.origin.orientation.w = qw

        T_inv = inverse_transform(T)
        origin_inv = inverse_transform(origin_T)
        aligned.data = [-1]*(grid_robot.info.width*grid_robot.info.height)
        for iy in range(grid_robot.info.height):
            for ix in range(grid_robot.info.width):
                cell_x = (ix + 0.5) * grid_robot.info.resolution
                cell_y = (iy + 0.5) * grid_robot.info.resolution

                wx = new_origin[0][0] * cell_x + new_origin[0][1] * cell_y + new_origin[0][2]
                wy = new_origin[1][0] * cell_x + new_origin[1][1] * cell_y + new_origin[1][2]

                ox = T_inv[0][0]*wx + T_inv[0][1]*wy + T_inv[0][2]
                oy = T_inv[1][0]*wx + T_inv[1][1]*wy + T_inv[1][2]

                cx = origin_inv[0][0] * ox + origin_inv[0][1] * oy + origin_inv[0][2]
                cy = origin_inv[1][0] * ox + origin_inv[1][1] * oy + origin_inv[1][2]

                sx = int(cx / grid_robot.info.resolution)
                sy = int(cy / grid_robot.info.resolution)

                if 0 <= sx < grid_robot.info.width and 0 <= sy < grid_robot.info.height:
                    aligned.data[iy*grid_robot.info.width+ix] = grid_robot.data[sy*grid_robot.info.width+sx]
        
        return aligned

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