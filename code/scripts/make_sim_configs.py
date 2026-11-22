#!/usr/bin/env python3
"""Конфигурации ORB-SLAM3, VINS-Fusion и OpenVINS для симулированных датчиков.

Берёт штатные конфигурации EuRoC (копируются из образов) и заменяет в них только
параметры датчиков: внутренние параметры камеры, расположение камеры
относительно IMU, частоты и шумы IMU, ускорение свободного падения. Из настроек
алгоритмов изменён только способ инициализации OpenVINS (см. ниже).

    scripts/make_sim_configs.py sim/generated/calib.yaml configs/sim
"""
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

TEMPLATES = [
    ("localhost/uav-orbslam3", "/opt/ORB_SLAM3/Examples/Monocular-Inertial/EuRoC.yaml", "orbslam3_mono_inertial.yaml"),
    ("localhost/uav-vins", "/opt/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml", "vins/mono_imu_config.yaml"),
    ("localhost/uav-vins", "/opt/catkin_ws/src/VINS-Fusion/config/euroc/cam0_pinhole.yaml", "vins/cam0_pinhole.yaml"),
    ("localhost/uav-openvins", "/opt/ws/src/open_vins/config/euroc_mav/estimator_config.yaml", "openvins/estimator_config.yaml"),
    ("localhost/uav-openvins", "/opt/ws/src/open_vins/config/euroc_mav/kalibr_imu_chain.yaml", "openvins/kalibr_imu_chain.yaml"),
    ("localhost/uav-openvins", "/opt/ws/src/open_vins/config/euroc_mav/kalibr_imucam_chain.yaml", "openvins/kalibr_imucam_chain.yaml"),
]
GRAVITY = 9.8  # Gazebo по умолчанию


def fetch(image, path):
    with tempfile.TemporaryDirectory() as d:
        subprocess.run(["podman", "run", "--rm", "--network=none", "-v", f"{d}:/o:Z", image, "cp", path, "/o/f"],
                       check=True)
        return Path(d, "f").read_text()


def sub(text, pattern, repl, count=1):
    new, n = re.subn(pattern, repl, text, count=count, flags=re.M | re.S)
    if n == 0:
        raise SystemExit(f"шаблон не найден: {pattern}")
    return new


def matrix_rows(T, indent):
    return "\n".join(f"{indent}- [{', '.join(f'{v:.9f}' for v in row)}]" for row in T)


def main(calib_path, out_dir):
    c = yaml.safe_load(Path(calib_path).read_text())
    cam, imu = c["cam0"], c["imu0"]
    T = [c["T_BS"][i * 4:(i + 1) * 4] for i in range(4)]
    flat = ", ".join(f"{v:.9f}" for v in c["T_BS"])
    out = Path(out_dir)

    for image, src, dst in TEMPLATES:
        t = fetch(image, src)
        if dst.startswith("orbslam3"):
            for k, v in (("fx", cam["fx"]), ("fy", cam["fy"]), ("cx", cam["cx"]), ("cy", cam["cy"])):
                t = sub(t, rf"^Camera1\.{k}:.*?$", f"Camera1.{k}: {v}")
            for k in ("k1", "k2", "p1", "p2"):
                t = sub(t, rf"^Camera1\.{k}:.*?$", f"Camera1.{k}: 0.0")
            t = sub(t, r"^Camera\.newWidth:.*?\n", "")  # штатное уменьшение кадра подобрано под EuRoC
            t = sub(t, r"^Camera\.newHeight:.*?\n", "")
            t = sub(t, r"^Camera\.fps:.*?$", f"Camera.fps: {cam['rate_hz']}")
            t = sub(t, r"(IMU\.T_b_c1:.*?data:\s*)\[[^\]]*\]", rf"\g<1>[{flat}]")
            t = sub(t, r"^IMU\.NoiseGyro:.*?$", f"IMU.NoiseGyro: {imu['gyroscope_noise_density']}")
            t = sub(t, r"^IMU\.NoiseAcc:.*?$", f"IMU.NoiseAcc: {imu['accelerometer_noise_density']}")
            t = sub(t, r"^IMU\.GyroWalk:.*?$", f"IMU.GyroWalk: {imu['gyroscope_random_walk']}")
            t = sub(t, r"^IMU\.AccWalk:.*?$", f"IMU.AccWalk: {imu['accelerometer_random_walk']}")
            t = sub(t, r"^IMU\.Frequency:.*?$", f"IMU.Frequency: {float(imu['rate_hz'])}")
        elif dst == "vins/mono_imu_config.yaml":
            t = sub(t, r'^cam0_calib:.*?$', 'cam0_calib: "cam0_pinhole.yaml"')
            t = sub(t, r"(body_T_cam0:.*?data:\s*)\[[^\]]*\]", rf"\g<1>[{flat}]")
            t = sub(t, r"^g_norm:.*?$", f"g_norm: {GRAVITY}")
            t = sub(t, r"^image_width:.*?$", f"image_width: {cam['width']}")
            t = sub(t, r"^image_height:.*?$", f"image_height: {cam['height']}")
        elif dst == "vins/cam0_pinhole.yaml":
            for k in ("k1", "k2", "p1", "p2"):
                t = sub(t, rf"^(\s*){k}:.*?$", rf"\g<1>{k}: 0.0")
            for k in ("fx", "fy", "cx", "cy"):
                t = sub(t, rf"^(\s*){k}:.*?$", rf"\g<1>{k}: {cam[k]}")
        elif dst == "openvins/estimator_config.yaml":
            t = sub(t, r"^gravity_mag:.*?$", f"gravity_mag: {GRAVITY}")
            # динамическая инициализация: статической нужен неподвижный аппарат и затем
            # рывок, а висящий коптер не бывает неподвижен (мера возбуждения IMU при
            # висении ~0,5, при начале движения ~0,7 -- надёжного порога нет)
            t = sub(t, r"^init_dyn_use:\s*\w+", "init_dyn_use: true")
        elif dst == "openvins/kalibr_imu_chain.yaml":
            for k in ("accelerometer_noise_density", "accelerometer_random_walk",
                      "gyroscope_noise_density", "gyroscope_random_walk"):
                t = sub(t, rf"^(\s*){k}:.*?$", rf"\g<1>{k}: {imu[k]}")
            t = sub(t, r"^(\s*)update_rate:.*?$", rf"\g<1>update_rate: {float(imu['rate_hz'])}")
        elif dst == "openvins/kalibr_imucam_chain.yaml":
            t = sub(t, r"\ncam1:.*$", "\n")  # одна камера
            t = sub(t, r"(cam0:\s*\n\s*T_imu_cam:[^\n]*\n)(\s*- \[[^\n]*\n){4}", r"\g<1>" + matrix_rows(T, "    ") + "\n")
            t = sub(t, r"cam_overlaps: \[1\]", "cam_overlaps: []")
            t = sub(t, r"distortion_coeffs: \[[^\]]*\]", "distortion_coeffs: [0.0, 0.0, 0.0, 0.0]")
            t = sub(t, r"intrinsics: \[[^\]]*\]", f"intrinsics: [{cam['fx']}, {cam['fy']}, {cam['cx']}, {cam['cy']}]")
            t = sub(t, r"resolution: \[[^\]]*\]", f"resolution: [{cam['width']}, {cam['height']}]")
        p = out / dst
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(t)
        print(p)


if __name__ == "__main__":
    main(*sys.argv[1:3])
