import os
import sys
import math

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import csv

RESULTS_DIR = sys.argv[1]
BAG_PATH = os.path.join(RESULTS_DIR, "bag")

TOPICS = {
    "/artbot1/gt_pose_rel": "artbot1_gt.csv",
    "/artbot1/odom": "artbot1_odom.csv",

    "/artbot2/gt_pose_rel": "artbot2_gt.csv",
    "/artbot2/odom": "artbot2_odom.csv",
}

TF_TOPIC = "/tf"
TF_TOPIC_STATIC = "/tf_static"

FRAMES = {
    "artbot1": "artbot1_slam.csv",
    "artbot2": "artbot2_slam.csv",
}



# ----
# 2D Transform Utils
# ----
def quat_to_yaw(q):

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(siny_cosp, cosy_cosp)


def compose_2d(t1, t2):

    x1, y1, yaw1 = t1
    x2, y2, yaw2 = t2

    x = x1 + math.cos(yaw1) * x2 - math.sin(yaw1) * y2
    y = y1 + math.sin(yaw1) * x2 + math.cos(yaw1) * y2

    yaw = yaw1 + yaw2

    return (x, y, yaw)

# ----
# Rosbag Reader Setup
# ----
reader = SequentialReader()

reader.open(
    StorageOptions(
        uri=BAG_PATH,
        storage_id='sqlite3'
    ),
    ConverterOptions('', '')
)

topic_types = reader.get_all_topics_and_types()
type_map = {
    topic.name: topic.type
    for topic in topic_types
}

# ----
# CSV Setup
# ----

# GT and Odom
writers = {}
files = {}

for topic, filename in TOPICS.items():
    csv_path = os.path.join(RESULTS_DIR, filename)
    f = open(csv_path, "w", newline="")
    writer = csv.writer(f)

    writer.writerow([
        "timestamp",
        "x",
        "y"
    ])

    files[topic] = f
    writers[topic] = writer

# SLAM
slam_writers = {}

for robot, filename in FRAMES.items():
    csv_path = os.path.join(RESULTS_DIR, filename)
    f = open(csv_path, "w", newline="")
    writer = csv.writer(f)

    writer.writerow([
        "timestamp",
        "x",
        "y"
    ])

    files[robot] = f
    slam_writers[robot] = writer

# ----
# TF Tree Storage
# ----
tf_tree = {}

def store_transform(transform):
    parent = transform.header.frame_id
    child = transform.child_frame_id

    x = transform.transform.translation.x
    y = transform.transform.translation.y

    yaw = quat_to_yaw(transform.transform.rotation)

    tf_tree[(parent, child)] = (x, y, yaw)

# ----
# Main Loop
# ----

while reader.has_next():

    topic, data, timestamp = reader.read_next()

    # GT and Odom
    if topic in TOPICS:
        
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        writers[topic].writerow([
            timestamp * 1e-9,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
    
    # SLAM
    elif topic == TF_TOPIC or topic == TF_TOPIC_STATIC:

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        for transform in msg.transforms:
            store_transform(transform=transform)

            parent = transform.header.frame_id
            child = transform.child_frame_id

        for robot in FRAMES.keys():

            map_frame = f'{robot}_map'
            odom_frame = f'{robot}_odom'

            if parent == map_frame and child == odom_frame:

                try:
                    t_map_odom = tf_tree[(f'{robot}_map', f'{robot}_odom')]
                    t_odom_base = tf_tree[(f'{robot}_odom', f'{robot}_base_link')]
                    t_base_footprint = tf_tree[(f'{robot}_base_link', f'{robot}_base_footprint')]

                    pose = compose_2d(compose_2d(t_map_odom, t_odom_base), t_base_footprint)
                    x, y, yaw = pose

                    slam_writers[robot].writerow([
                        timestamp * 1e-9,
                        x,
                        y
                    ])
                except KeyError:
                    pass

# ----
# Close CSV Files
# ----
for f in files.values():
    f.close()

print("Arquivos gerados:")

for filename in TOPICS.values():
    print(f"  - {filename}")

for filename in FRAMES.values():
    print(f"  - {filename}")