#!/usr/bin/env python3
"""Сообщения nav_msgs/Odometry из bag-файла ROS 1 -> траектория в формате TUM.

    scripts/odom_bag_to_tum.py vins_online.bag out.tum [топик]
"""
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader


def main(bag, dst, topic_suffix="odometry_rect"):
    n = 0
    with AnyReader([Path(bag)]) as reader, open(dst, "w") as fo:
        conns = [c for c in reader.connections if c.topic.endswith(topic_suffix)]
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            fo.write(f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")
            n += 1
    print(f"{dst}: {n} поз")


if __name__ == "__main__":
    main(*sys.argv[1:])
