#!/usr/bin/env python3
"""Запись данных симуляции в формате EuRoC (ASL) по топикам Gazebo.

    python3 record_gz.py /out/mav0 [--start-file /out/record.flag]

    mav0/cam0/data/<t_ns>.png, mav0/cam0/data.csv
    mav0/imu0/data.csv                         -- t, w_xyz [рад/с], a_xyz [м/с^2]
    mav0/state_groundtruth_estimate0/data.csv  -- t, p_xyz, q_wxyz, v_xyz, нули смещений
Время -- время симуляции. Запись идёт, пока существует --start-file
(если он задан) и до SIGINT/SIGTERM.
"""
import argparse
import queue
import signal
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec


class Recorder:
    def __init__(self, root, flag):
        self.root, self.flag = root, flag
        (root / "cam0/data").mkdir(parents=True, exist_ok=True)
        (root / "imu0").mkdir(parents=True, exist_ok=True)
        (root / "state_groundtruth_estimate0").mkdir(parents=True, exist_ok=True)
        self.cam = open(root / "cam0/data.csv", "w")
        self.cam.write("#timestamp [ns],filename\n")
        self.imu = open(root / "imu0/data.csv", "w")
        self.imu.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                       "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        self.gt = open(root / "state_groundtruth_estimate0/data.csv", "w")
        self.gt.write("#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], "
                      "v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1], b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], "
                      "b_w_RS_S_z [rad s^-1], b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]\n")
        self.lock = threading.Lock()
        self.images = queue.Queue(maxsize=2000)
        self.counts = {"cam": 0, "imu": 0, "gt": 0}
        self.last = {"cam": -1, "imu": -1, "gt": -1}

    def active(self):
        return self.flag is None or self.flag.exists()

    def _fresh(self, kind, t):
        # Gazebo может повторно прислать сообщение с тем же временем
        if t <= self.last[kind]:
            return False
        self.last[kind] = t
        return True

    def on_image(self, msg):
        if not self.active():
            return
        t = stamp_ns(msg)
        if not self._fresh("cam", t):
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        self.images.put((t, img.copy()))

    def on_imu(self, msg):
        if not self.active():
            return
        t = stamp_ns(msg)
        if not self._fresh("imu", t):
            return
        w, a = msg.angular_velocity, msg.linear_acceleration
        with self.lock:
            self.imu.write(f"{t},{w.x:.9f},{w.y:.9f},{w.z:.9f},{a.x:.9f},{a.y:.9f},{a.z:.9f}\n")
            self.counts["imu"] += 1

    def on_gt(self, msg):
        if not self.active():
            return
        t = stamp_ns(msg)
        if not self._fresh("gt", t):
            return
        p, q, v = msg.pose.position, msg.pose.orientation, msg.twist.linear
        with self.lock:
            # скорость в odometry -- в системе корпуса; переводим в мировую
            R = quat_to_R(q.w, q.x, q.y, q.z)
            vw = R @ np.array([v.x, v.y, v.z])
            self.gt.write(f"{t},{p.x:.6f},{p.y:.6f},{p.z:.6f},{q.w:.7f},{q.x:.7f},{q.y:.7f},{q.z:.7f},"
                          f"{vw[0]:.6f},{vw[1]:.6f},{vw[2]:.6f},0,0,0,0,0,0\n")
            self.counts["gt"] += 1

    def writer(self, stop):
        while not (stop.is_set() and self.images.empty()):
            try:
                t, img = self.images.get(timeout=0.2)
            except queue.Empty:
                continue
            cv2.imwrite(str(self.root / f"cam0/data/{t}.png"), img)
            with self.lock:
                self.cam.write(f"{t},{t}.png\n")
                self.counts["cam"] += 1

    def close(self):
        for f in (self.cam, self.imu, self.gt):
            f.close()


def quat_to_R(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("--start-file", type=Path)
    ap.add_argument("--cam-topic", default="/cam0/image")
    ap.add_argument("--imu-topic", default="/imu0")
    ap.add_argument("--gt-topic", default="/ground_truth")
    args = ap.parse_args()

    rec = Recorder(args.root, args.start_file)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    node = Node()
    for topic, typ, cb in ((args.cam_topic, Image, rec.on_image), (args.imu_topic, IMU, rec.on_imu),
                           (args.gt_topic, Odometry, rec.on_gt)):
        if not node.subscribe(typ, topic, cb):
            raise SystemExit(f"не удалось подписаться на {topic}")
    th = threading.Thread(target=rec.writer, args=(stop,))
    th.start()
    last = time.time()
    while not stop.is_set():
        time.sleep(0.5)
        if time.time() - last > 10:
            last = time.time()
            print(f"запись: {rec.counts}", flush=True)
    th.join()
    rec.close()
    print(f"итого: {rec.counts}", flush=True)


if __name__ == "__main__":
    main()
