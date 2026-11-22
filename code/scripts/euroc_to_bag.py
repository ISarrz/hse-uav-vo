#!/usr/bin/env python3
"""Конвертация последовательности EuRoC (формат ASL) в rosbag для ROS 1 и ROS 2.

Топики совпадают с оригинальными bag-файлами EuRoC, поэтому штатные конфиги
VINS-Fusion и OpenVINS работают без изменений:
    /cam0/image_raw, /cam1/image_raw  sensor_msgs/Image (mono8)
    /imu0                             sensor_msgs/Imu

    scripts/euroc_to_bag.py data/euroc/MH_01_easy --ros1 --ros2
"""
import argparse
import csv
from pathlib import Path

import numpy as np
from PIL import Image as PILImage
from rosbags.rosbag1 import Writer as Writer1
from rosbags.rosbag2 import Writer as Writer2
from rosbags.typesys import Stores, get_typestore


def read_csv(path):
    with open(path) as f:
        return [row for row in csv.reader(f) if row and not row[0].startswith("#")]


def build_messages(ts, seq_dir):
    """Генератор (время_нс, топик, тип, сообщение), отсортированный по времени."""
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    Imu = ts.types["sensor_msgs/msg/Imu"]
    Image = ts.types["sensor_msgs/msg/Image"]
    Vector3 = ts.types["geometry_msgs/msg/Vector3"]
    Quaternion = ts.types["geometry_msgs/msg/Quaternion"]

    ros1 = "seq" in Header.__dataclass_fields__

    def header(t_ns, frame):
        stamp = Time(sec=t_ns // 10**9, nanosec=t_ns % 10**9)
        if ros1:
            return Header(seq=0, stamp=stamp, frame_id=frame)
        return Header(stamp=stamp, frame_id=frame)

    events = []
    for row in read_csv(seq_dir / "mav0/imu0/data.csv"):
        events.append((int(row[0]), "imu", row))
    for cam in ("cam0", "cam1"):
        if not (seq_dir / f"mav0/{cam}/data.csv").exists():
            continue  # в симуляции камера одна
        for row in read_csv(seq_dir / f"mav0/{cam}/data.csv"):
            events.append((int(row[0]), cam, row))
    events.sort(key=lambda e: e[0])

    zero_cov = np.zeros(9, dtype=np.float64)
    for t, kind, row in events:
        if kind == "imu":
            wx, wy, wz, ax, ay, az = map(float, row[1:7])
            msg = Imu(
                header=header(t, "imu4"),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                orientation_covariance=np.array([-1.0] + [0.0] * 8),
                angular_velocity=Vector3(x=wx, y=wy, z=wz),
                angular_velocity_covariance=zero_cov,
                linear_acceleration=Vector3(x=ax, y=ay, z=az),
                linear_acceleration_covariance=zero_cov,
            )
            yield t, "/imu0", Imu.__msgtype__, msg
        else:
            img = np.asarray(PILImage.open(seq_dir / f"mav0/{kind}/data/{row[1]}").convert("L"))
            msg = Image(
                header=header(t, kind),
                height=img.shape[0], width=img.shape[1], encoding="mono8",
                is_bigendian=0, step=img.shape[1], data=img.reshape(-1),
            )
            yield t, f"/{kind}/image_raw", Image.__msgtype__, msg


def write_bag(seq_dir, path, ros2):
    ts = get_typestore(Stores.ROS2_HUMBLE if ros2 else Stores.ROS1_NOETIC)
    writer = Writer2(path, version=8) if ros2 else Writer1(path)
    conns = {}
    with writer as w:
        for t, topic, msgtype, msg in build_messages(ts, seq_dir):
            if topic not in conns:
                kw = {"typestore": ts}
                conns[topic] = w.add_connection(topic, msgtype, **kw)
            data = ts.serialize_cdr(msg, msgtype) if ros2 else ts.serialize_ros1(msg, msgtype)
            w.write(conns[topic], t, data)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("seq_dir", type=Path)
    ap.add_argument("--ros1", action="store_true")
    ap.add_argument("--ros2", action="store_true")
    args = ap.parse_args()
    name = args.seq_dir.name
    if args.ros1:
        out = args.seq_dir / f"{name}.bag"
        if not out.exists():
            write_bag(args.seq_dir, out, ros2=False)
        print(f"ROS 1: {out}")
    if args.ros2:
        out = args.seq_dir / f"{name}_ros2"
        if not out.exists():
            write_bag(args.seq_dir, out, ros2=True)
        print(f"ROS 2: {out}")


if __name__ == "__main__":
    main()
