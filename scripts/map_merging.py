#!/usr/bin/env python3

import math
import numpy as np
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseArray, TransformStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid

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

def pose2matrix(tx, ty, teta):
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
    la, ca = len(A), len(A[0])
    lb, cb = len(B), len(B[0])
    if ca != lb:
        raise ValueError("Invalid matrix multiplication")
    result = [[0.0 for _ in range(cb)] for _ in range(la)]
    for i in range(la):
        for j in range(cb):
            for k in range(ca):
                result[i][j] += A[i][k]*B[k][j]
    return result

def corners_coord(map_info):
    origin_T = pose2matrix(map_info.origin.position.x, map_info.origin.position.y, get_yaw(map_info.origin))
    corner1 = matrix_multiplication(origin_T, [[0], [0], [1]])
    corner2 = matrix_multiplication(origin_T, [[map_info.width*map_info.resolution], [0], [1]])
    corner3 = matrix_multiplication(origin_T, [[0], [map_info.height*map_info.resolution], [1]])
    corner4 = matrix_multiplication(origin_T, [[map_info.width*map_info.resolution], [map_info.height*map_info.resolution], [1]])
    
    return [corner1, corner2, corner3, corner4]

def compute_merged_corners(map1_info, map2_info):
    all_corners = corners_coord(map1_info) + corners_coord(map2_info)
    xs = [corner[0] for corner in all_corners]
    ys = [corner[1] for corner in all_corners]

    xmin = min(xs)
    xmax = max(xs)
    ymin = min(ys)
    ymax = max(ys)

    return (xmin[0], xmax[0], ymin[0], ymax[0])

def input_noise(gaussian_stddev):
    noise = np.random.normal(0.0, gaussian_stddev)
    return noise

class MapMerging(Node):
    def __init__(self):
        super().__init__("map_merge")

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

        self.declare_parameter(
            name='robot_names',
            value=['artbot1', 'artbot2']
        )
        self.robot_names = self.get_parameter(
            'robot_names'
        ).get_parameter_value().string_array_value

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Internal variables
        self.global_poses = {} # raw poses (gazebo world frame of reference) - map alignment
        self.local_poses = {} # odom poses (local odom frame of reference) - map alignment
        self.local_maps = {} # local individual maps (local map frame of reference) - map alignment
        self.local_aligned_maps = {} # local individual maps after alignment (global merged map frame of reference) - map fusion

        self.aligned_map_publishers = {} # aligned maps publishers
        self.tf_broadcaster = TransformBroadcaster(self) # tf brodcaster for new tfs and frames
        self.tf_local2merged = {} # stamped transform from local map frames to global merged map frame

        distance_stddev = self.gaussian_noise[0]
        theta_stddev = self.gaussian_noise[1]
        self.tx_noise = input_noise(distance_stddev)
        self.ty_noise = input_noise(distance_stddev)
        self.theta_noise = input_noise(theta_stddev)

        # Subscriptions and Publishers
        for robot in self.robot_names:
            global_topic = f"/{robot}/pose_raw"
            local_topic = f"/{robot}/slam_pose"
            local_map_topic = f"/{robot}/map"
            aligned_map_topic = f"/{robot}/map_aligned"

            self.create_subscription(
                PoseArray,
                global_topic,
                lambda msg, name=robot: self.gt_callback(msg, name),
                10
            )
            self.get_logger().info(f"Subscribed to {global_topic}")

            self.create_subscription(
                PoseStamped,
                local_topic,
                lambda msg, name=robot: self.pose_callback(msg, name),
                10
            )
            self.get_logger().info(f"Subscribed to {local_topic}")

            self.create_subscription(
                OccupancyGrid,
                local_map_topic,
                lambda msg, name=robot: self.map_callback(msg, name),
                10
            )
            self.get_logger().info(f"Subscribed to {local_map_topic}")

            self.aligned_map_publishers[robot] = self.create_publisher(
                OccupancyGrid,
                aligned_map_topic,
                map_qos,
            )
            self.get_logger().info(f"Publishing to {aligned_map_topic}")

        merged_map_topic = "/merged_map"
        self.merged_map_publisher = self.create_publisher(OccupancyGrid, merged_map_topic, map_qos)
        self.get_logger().info(f"Publishing to {merged_map_topic}")

        self.timer = self.create_timer(30.0, self.publish_transforms)

    def gt_callback(self, msg, robot):
        if not msg.poses:
            return
        self.global_poses[robot] = msg.poses[0]

    def pose_callback(self, msg, robot):
        self.local_poses[robot] = msg.pose

    def map_callback(self, msg, robot):
        self.local_maps[robot] = msg

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

    def publish_transforms(self):
        gt_poses_ok = len(self.global_poses) == len(self.robot_names)
        odom_poses_ok = len(self.local_poses) == len(self.robot_names)
        local_maps_ok = len(self.local_maps) == len(self.robot_names)
        reference_robot = self.robot_names[0]

        if (gt_poses_ok and odom_poses_ok and local_maps_ok):
            alignment_data = self.compute_alignment_data()
            transformations, noises = self.compute_transformations(alignment_data)
            self.local_aligned_maps[reference_robot], self.tf_local2merged[reference_robot] = self.publish_reference_tf(self.local_maps[reference_robot])
            
            for robot, T in transformations.items():
                self.tf_local2merged[robot] = self.publish_map_transform(robot, T)
                self.local_aligned_maps[robot] = self.map_alignment(
                    self.local_maps[robot], T
                )

            if len(self.local_aligned_maps) == len(self.robot_names):
                merged_map = self.local_aligned_maps[reference_robot]
                for robot in self.robot_names[1:]:
                    merged_map = self.map_fusion(merged_map, self.local_aligned_maps[robot])

                try:
                    for robot in self.robot_names:
                        if robot == reference_robot:
                            self.tf_broadcaster.sendTransform(self.tf_local2merged[robot])
                            self.aligned_map_publishers[robot].publish(self.local_aligned_maps[robot])
                        else:
                            self.tf_broadcaster.sendTransform(self.tf_local2merged[robot])
                            self.aligned_map_publishers[robot].publish(self.local_aligned_maps[robot])
                    self.merged_map_publisher.publish(merged_map)
                    self.save_alignment_data(alignment_data, transformations, noises)
                    self.get_logger().info(f"Aligned maps, transforms, merged map and json data published.")
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

        return aligned, tf

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

        return tf

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
            pose = self.local_poses[robot]

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
                    self.global_poses[robot_a], self.global_poses[robot_b]
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

            Tc = pose2matrix(
                current["translation"]["x"],
                current["translation"]["y"],
                current["rotation"]
            )
            To = pose2matrix(
                other["translation"]["x"],
                other["translation"]["y"],
                other["rotation"]
            )

            Tr_noise = pose2matrix(
                relative["translation"]["x"] + self.tx_noise,
                relative["translation"]["y"] + self.ty_noise,
                relative["rotation"] + self.theta_noise,
            )
            noise = [self.tx_noise, self.ty_noise, self.theta_noise]

            T = matrix_multiplication(
                matrix_multiplication(Tc, Tr_noise),
                inverse_transform(To)
            )

            transformations[robot] = T
            noises[robot] = noise
    
        return transformations, noises

    def map_alignment(self, grid_robot, T):
        # Setup map info
        aligned = OccupancyGrid()
        aligned.header.stamp = grid_robot.header.stamp
        aligned.header.frame_id = "merged_map" # global frame
        aligned.info.map_load_time = grid_robot.info.map_load_time
        aligned.info.resolution = grid_robot.info.resolution
        aligned.info.width = grid_robot.info.width
        aligned.info.height = grid_robot.info.height

        # Setup important transformations
        origin = grid_robot.info.origin
        origin_T = pose2matrix(
            origin.position.x, origin.position.y, get_yaw(origin)
        ) # artbotX_map -> cell inf esq
        new_origin_T = matrix_multiplication(T, origin_T) # merged_map -> cell inf esq

        # Set new origin from transformation
        tx, ty, yaw = matrix2pose(new_origin_T)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        aligned.info.origin.position.x = tx
        aligned.info.origin.position.y = ty
        aligned.info.origin.orientation.x = qx
        aligned.info.origin.orientation.y = qy
        aligned.info.origin.orientation.z = qz
        aligned.info.origin.orientation.w = qw

        # Copy map data
        aligned.data = grid_robot.data
        
        return aligned

    def map_fusion(self, map1, map2):
        # Setup map info
        merged_map = OccupancyGrid()
        merged_map.header = map1.header
        merged_map.info.resolution = min(map1.info.resolution, map2.info.resolution)

        # Determine an origin for the merged map from the 2 local maps
        min_x, max_x, min_y, max_y = compute_merged_corners(map1.info, map2.info)
        qx, qy, qz, qw = yaw_to_quaternion(yaw=0.0)
        merged_map.info.origin.position.x = min_x
        merged_map.info.origin.position.y = min_y
        merged_map.info.origin.orientation.x = qx
        merged_map.info.origin.orientation.y = qy
        merged_map.info.origin.orientation.z = qz
        merged_map.info.origin.orientation.w = qw

        # Determine new width and height with the new origin
        merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
        merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))

        # Setup important transformations
        map1_origin_T = pose2matrix(map1.info.origin.position.x, map1.info.origin.position.y, get_yaw(map1.info.origin))
        map2_origin_T = pose2matrix(map2.info.origin.position.x, map2.info.origin.position.y, get_yaw(map2.info.origin))
        map1_origin_Tinv = inverse_transform(map1_origin_T)
        map2_origin_Tinv = inverse_transform(map2_origin_T)
        global_origin_T = pose2matrix(merged_map.info.origin.position.x, merged_map.info.origin.position.y, get_yaw(merged_map.info.origin))

        # Initialize merged map data vector all with unknown cells
        merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)

        # Copy map1 to merged map
        for merged_y in range(merged_map.info.height):
            for merged_x in range(merged_map.info.width):
                merged_i = merged_x + merged_y * merged_map.info.width
                merged_cell_x = (merged_x + 0.5)*merged_map.info.resolution
                merged_cell_y = (merged_y + 0.5)*merged_map.info.resolution
                wx = merged_cell_x*global_origin_T[0][0] + merged_cell_y*global_origin_T[0][1] + global_origin_T[0][2]
                wy = merged_cell_x*global_origin_T[1][0] + merged_cell_y*global_origin_T[1][1] + global_origin_T[1][2]
                cell_x = map1_origin_Tinv[0][0] * wx + map1_origin_Tinv[0][1] * wy + map1_origin_Tinv[0][2]
                cell_y = map1_origin_Tinv[1][0] * wx + map1_origin_Tinv[1][1] * wy + map1_origin_Tinv[1][2]
                x = int(math.floor(cell_x/merged_map.info.resolution))
                y = int(math.floor(cell_y/merged_map.info.resolution))
                i = x + y * map1.info.width

                is_within_bounds = (0<=x<map1.info.width) and (0<=y<map1.info.height)
                if is_within_bounds: 
                    merged_map.data[merged_i] = map1.data[i]

        # Copy map2 to merged map
        for merged_y in range(merged_map.info.height):
            for merged_x in range(merged_map.info.width):
                merged_i = merged_x + merged_y * merged_map.info.width
                merged_cell_x = (merged_x + 0.5)*merged_map.info.resolution
                merged_cell_y = (merged_y + 0.5)*merged_map.info.resolution
                wx = merged_cell_x*global_origin_T[0][0] + merged_cell_y*global_origin_T[0][1] + global_origin_T[0][2]
                wy = merged_cell_x*global_origin_T[1][0] + merged_cell_y*global_origin_T[1][1] + global_origin_T[1][2]
                cell_x = map2_origin_Tinv[0][0] * wx + map2_origin_Tinv[0][1] * wy + map2_origin_Tinv[0][2]
                cell_y = map2_origin_Tinv[1][0] * wx + map2_origin_Tinv[1][1] * wy + map2_origin_Tinv[1][2]
                x = int(math.floor(cell_x/merged_map.info.resolution))
                y = int(math.floor(cell_y/merged_map.info.resolution))
                i = x + y * map2.info.width

                is_within_bounds = (0<=x<map2.info.width) and (0<=y<map2.info.height)
                if is_within_bounds and (merged_map.data[merged_i] == -1): 
                    merged_map.data[merged_i] = map2.data[i]
                
        return merged_map

def main(args=None):

    rclpy.init(args=args)

    node = MapMerging()

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