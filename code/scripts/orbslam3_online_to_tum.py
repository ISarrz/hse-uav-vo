#!/usr/bin/env python3
"""Онлайн-траектория ORB-SLAM3 (поза камеры) -> траектория корпуса (IMU) в формате TUM.

    scripts/orbslam3_online_to_tum.py online_trajectory.txt EuRoC.yaml out.tum

Оставляются кадры с состоянием трекинга OK (2) и RECENTLY_LOST (3): в последнем
поза прогнозируется по IMU, и именно её система выдала бы потребителю.
Twb = Twc * inv(Tbc), где Tbc = IMU.T_b_c1 из конфига ORB-SLAM3.
"""
import re
import sys

import numpy as np
from scipy.spatial.transform import Rotation

GOOD_STATES = {2, 3}


def read_tbc(cfg_path):
    text = open(cfg_path).read()
    m = re.search(r"IMU\.T_b_c1:.*?data:\s*\[([^\]]*)\]", text, re.S)
    return np.array([float(v) for v in m.group(1).replace("\n", " ").split(",")]).reshape(4, 4)


def main(src, cfg, dst):
    T_cb = np.linalg.inv(read_tbc(cfg))
    kept = 0
    with open(src) as fi, open(dst, "w") as fo:
        for line in fi:
            v = line.split()
            if len(v) != 9 or int(v[1]) not in GOOD_STATES:
                continue
            T_wc = np.eye(4)
            T_wc[:3, :3] = Rotation.from_quat([float(x) for x in v[5:9]]).as_matrix()
            T_wc[:3, 3] = [float(x) for x in v[2:5]]
            T_wb = T_wc @ T_cb
            q = Rotation.from_matrix(T_wb[:3, :3]).as_quat()
            p = T_wb[:3, 3]
            fo.write(f"{v[0]} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")
            kept += 1
    print(f"{dst}: {kept} поз")


if __name__ == "__main__":
    main(*sys.argv[1:4])
