#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
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
        self.local_maps = {}
        self.map_publishers = {}

        first_robot = self.robot_names[0]
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
            
            if robot is not first_robot:
                self.map_publishers[robot] = self.create_publisher(
                    OccupancyGrid,
                    aligned_map_topic,
                    10
                )
                self.get_logger().info(f"Publishing to {aligned_map_topic}")

    def gt_callback(self, msg, robot):
        if not msg.poses:
            return
        self.gt_poses[robot] = msg.poses[0]

    def odom_callback(self, msg, robot):
        self.odom_poses[robot] = msg.pose.pose

    def map_callback(self, msg, robot):
        self.local_maps[robot] = msg
        self.check_ready()

    def check_ready(self):
        gt_poses_ok = len(self.gt_poses) == len(self.robot_names)
        odom_poses_ok = len(self.odom_poses) == len(self.robot_names)
        local_maps_ok = len(self.local_maps) == len(self.robot_names)

        if (gt_poses_ok and odom_poses_ok and local_maps_ok):
            alignment_data = self.compute_alignment_data()
            transformations = self.compute_transformations(alignment_data)

            for robot, T in transformations.items():
                aligned = self.map_alignment(
                    self.local_maps[self.robot_names[0]], self.local_maps[robot], T
                )

                self.map_publishers[robot].publish(aligned)

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

    def map_alignment(self, grid_ref, grid_robot, T):
        aligned = OccupancyGrid()

        aligned.header.stamp = grid_robot.header.stamp
        aligned.header.frame_id = grid_ref.header.frame_id

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

        aligned.data = [-1]*(grid_robot.info.width*grid_robot.info.height)
        ox = grid_robot.info.origin.position.x
        oy = grid_robot.info.origin.position.y

        for y in range(grid_robot.info.height):
            for x in range(grid_robot.info.width):
                idx = y*grid_robot.info.width + x
                value = grid_robot.data[idx]

                cell_x = (x + 0.5) * grid_robot.info.resolution
                cell_y = (y + 0.5) * grid_robot.info.resolution

                wx = origin_T[0][0] * cell_x + origin_T[0][1] * cell_y + origin_T[0][2]
                wy = origin_T[1][0] * cell_x + origin_T[1][1] * cell_y + origin_T[1][2]

                nx = T[0][0]*wx + T[0][1]*wy + T[0][2]
                ny = T[1][0]*wx + T[1][1]*wy + T[1][2]
                new_origin_inv = inverse_transform(new_origin)
                cx = new_origin_inv[0][0] * nx + new_origin_inv[0][1] * ny + new_origin_inv[0][2]
                cy = new_origin_inv[1][0] * nx + new_origin_inv[1][1] * ny + new_origin_inv[1][2]

                ix = round(cx / grid_robot.info.resolution - 0.5)
                iy = round(cy / grid_robot.info.resolution - 0.5)

                if 0 <= ix < grid_robot.info.width and 0 <= iy < grid_robot.info.height:

                    aligned.data[iy*grid_robot.info.width+ix] = value
        
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