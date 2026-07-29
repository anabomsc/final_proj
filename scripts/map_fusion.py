#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid
import numpy as np

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import math

from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy._rclpy_pybind11 import RCLError

class MapFusion(Node):
    def __init__(self):
        super().__init__('map_fusion')

        self.robot_names = [
            "artbot1",
            "artbot2"
        ]

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.local_maps = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        for robot in self.robot_names:
            local_map_topic = f"/{robot}/map_aligned"
            self.create_subscription(
                OccupancyGrid,
                local_map_topic,
                lambda msg, name=robot: self.map_callback(msg, name),
                10
            )
            self.get_logger().info(f"Subscribed to {local_map_topic}")

        self.merged_map_publisher = self.create_publisher(OccupancyGrid, '/merged_map', map_qos)
        self.get_logger().info(f"Publishing to /merged_map")

    def map_callback(self, msg, robot):
        self.local_maps[robot] = msg
        self.fuse_all_maps()

    def fuse_all_maps(self):
        if len(self.local_maps) == len(self.robot_names):
            merged_map = self.local_maps[self.robot_names[0]]
            for robot in self.robot_names[1:]:
                merged_map = self.map_fusion(merged_map, self.local_maps[robot])

            try:
                self.merged_map_publisher.publish(merged_map)
            except RCLError:
                return

    def get_transform(self, source_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                "merged_map",
                source_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(str(e))
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        c = math.cos(yaw)
        s = math.sin(yaw)

        return {
            "tx": t.x,
            "ty": t.y,
            "c": c,
            "s": s
        }

    def transform_point(self, x, y, tf):
        X = tf["c"] * x - tf["s"] * y + tf["tx"]
        Y = tf["s"] * x + tf["c"] * y + tf["ty"]
        return X, Y

    def map_fusion(self, map1, map2):
        merged_map = OccupancyGrid()
        merged_map.header = map1.header
        merged_map.header.frame_id = 'merged_map'
        
        min_x = min(map1.info.origin.position.x, map2.info.origin.position.x)
        min_y = min(map1.info.origin.position.y, map2.info.origin.position.y)
        max_x = max(map1.info.origin.position.x + (map1.info.width * map1.info.resolution),
                    map2.info.origin.position.x + (map2.info.width * map2.info.resolution))
        max_y = max(map1.info.origin.position.y + (map1.info.height * map1.info.resolution),
                    map2.info.origin.position.y + (map2.info.height * map2.info.resolution))
        
        merged_map.info.origin.position.x = min_x
        merged_map.info.origin.position.y = min_y
        merged_map.info.resolution = min(map1.info.resolution, map2.info.resolution)
        merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
        merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
        
        merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
        for y in range(map1.info.height):
            for x in range(map1.info.width):
                i = x + y * map1.info.width
                merged_x = int(np.floor((map1.info.origin.position.x + x * map1.info.resolution - min_x) / merged_map.info.resolution))
                merged_y = int(np.floor((map1.info.origin.position.y + y * map1.info.resolution - min_y) / merged_map.info.resolution))
                merged_i = merged_x + merged_y * merged_map.info.width
                merged_map.data[merged_i] = map1.data[i]
        for y in range(map2.info.height):
            for x in range(map2.info.width):
                i = x + y * map2.info.width
                merged_x = int(np.floor((map2.info.origin.position.x + x * map2.info.resolution - min_x) / merged_map.info.resolution))
                merged_y = int(np.floor((map2.info.origin.position.y + y * map2.info.resolution - min_y) / merged_map.info.resolution))
                merged_i = merged_x + merged_y * merged_map.info.width
                if merged_map.data[merged_i] == -1:
                    merged_map.data[merged_i] = map2.data[i]
        return merged_map

def main(args=None):

    rclpy.init(args=args)

    node = MapFusion()

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