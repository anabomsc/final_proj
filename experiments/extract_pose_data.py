import os
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import csv

RESULTS_DIR = sys.argv[1]
BAG_PATH = os.path.join(RESULTS_DIR, "bag")

TOPICS = {
    "/artbot1/gt_odom": "artbot1_gt.csv",
    "/artbot1/odom": "artbot1_odom.csv",
    "/artbot1/slam_pose": "artbot1_slam.csv",

    "/artbot2/gt_odom": "artbot2_gt.csv",
    "/artbot2/odom": "artbot2_odom.csv",
    "/artbot2/slam_pose": "artbot2_slam.csv",
}

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

# ----
# Main Loop
# ----
while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic in TOPICS:
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if isinstance(msg, Odometry): # GT and Odom
            writers[topic].writerow([
                timestamp * 1e-9,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            ])

        if isinstance(msg, PoseStamped): # SLAM
            writers[topic].writerow([
                timestamp * 1e-9,
                msg.pose.position.x,
                msg.pose.position.y
            ])

# ----
# Close CSV Files
# ----
for f in files.values():
    f.close()

print("Arquivos gerados:")
for filename in TOPICS.values():
    print(f"  - {filename}")