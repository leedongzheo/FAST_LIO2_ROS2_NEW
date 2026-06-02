#!/usr/bin/env python3
import sys
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

if len(sys.argv) < 4:
    print("Usage: python3 export_odom_to_csv.py <bag_dir> <topic> <out_csv>")
    sys.exit(1)

bag_dir = sys.argv[1]
topic_name = sys.argv[2]
out_csv = sys.argv[3]

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap")
converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr"
)
reader.open(storage_options, converter_options)

topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

if topic_name not in topic_types:
    print("Topic not found:", topic_name)
    print("Available topics:")
    for k in topic_types:
        print(" ", k)
    sys.exit(1)

msg_type = get_message(topic_types[topic_name])

with open(out_csv, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])

    while reader.has_next():
        topic, data, bag_time = reader.read_next()
        if topic != topic_name:
            continue

        msg = deserialize_message(data, msg_type)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        writer.writerow([
            stamp,
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w
        ])

print("Saved:", out_csv)
