#!/usr/bin/env python3
"""Генерация сцены для симуляции: модель коптера с камерой и IMU и миры с разной поверхностью.

Запускается в контейнере uav-sim (нужны numpy, opencv и модели ardupilot_gazebo):
    python3 /sim/make_scene.py --out /sim/generated

Результат:
    generated/models/iris_vio/    -- Iris c ArduPilot + камера вниз + IMU + истинная поза
    generated/textures/<тип>.png  -- текстура поверхности
    generated/worlds/<тип>.sdf    -- мир с этой поверхностью
    generated/calib.yaml          -- параметры камеры и IMU для настройки VIO
"""
import argparse
import math
import re
from pathlib import Path

import cv2
import numpy as np

AG = Path("/opt/ardupilot_gazebo/models")

# ---------- датчики (по образцу EuRoC: MT9V034 + ADIS16448) ----------
CAM = dict(width=752, height=480, hfov=1.375, rate=20)  # fx = fy ~ 458 пкс
IMU_RATE = 200
# плотности шума ADIS16448 (как в конфигурациях EuRoC)
GYRO_ND, GYRO_RW = 1.6968e-4, 1.9393e-5   # рад/с/sqrt(Гц), рад/с^2/sqrt(Гц)
ACC_ND, ACC_RW = 2.0e-3, 3.0e-3           # м/с^2/sqrt(Гц), м/с^3/sqrt(Гц)
# крепление камеры: 10 см вперёд, 5 см вниз от центра, оптическая ось вниз,
# верх изображения смотрит вперёд по курсу
CAM_POSE = (0.10, 0.0, -0.05, 0.0, math.pi / 2, 0.0)

GROUND_SIZE = 600.0  # м
TARGET_MEAN = 100.0  # средняя яркость текстуры (0-255)
TEX_PX = 8192        # 7,3 см на пиксель


# ---------- текстуры ----------

def noise(rng, size, base, octaves, persistence=0.5):
    """Фрактальный шум: сумма увеличенных случайных сеток."""
    out = np.zeros((size, size), np.float32)
    amp, total = 1.0, 0.0
    for i in range(octaves):
        n = base * 2 ** i
        if n >= size:
            break
        g = rng.random((n, n)).astype(np.float32)
        out += amp * cv2.resize(g, (size, size), interpolation=cv2.INTER_CUBIC)
        total += amp
        amp *= persistence
    out /= total
    return (out - out.min()) / (out.max() - out.min() + 1e-9)


def to_px(m):
    return int(round(m / GROUND_SIZE * TEX_PX))


def draw_trees(img, rng, count, r_m, colors):
    for _ in range(count):
        c = (int(rng.integers(0, TEX_PX)), int(rng.integers(0, TEX_PX)))
        r = to_px(rng.uniform(*r_m))
        col = np.array(colors[rng.integers(len(colors))], np.float32) * rng.uniform(0.8, 1.15)
        # тень со стороны, противоположной солнцу, затем крона
        cv2.circle(img, (c[0] + r // 3, c[1] + r // 3), r, (25, 35, 20), -1, cv2.LINE_AA)
        cv2.circle(img, c, r, tuple(float(v) for v in col), -1, cv2.LINE_AA)
        cv2.circle(img, (c[0] - r // 3, c[1] - r // 3), max(r // 3, 1),
                   tuple(float(min(v * 1.25, 255)) for v in col), -1, cv2.LINE_AA)


def texture_mixed(rng):
    """Сельская местность: поля разных культур, дороги, лесополосы и отдельные деревья."""
    s = TEX_PX
    img = np.zeros((s, s, 3), np.float32)
    # поля -- ячейки Вороного вокруг случайных центров
    n_fields = 140
    centers = rng.random((n_fields, 2)) * s
    step = 2  # шаг сетки разбиения, пкс (~15 см): границы полей без заметных ступенек
    yy, xx = np.mgrid[0:s:step, 0:s:step].astype(np.float32)
    best = np.full(yy.shape, np.inf, np.float32)
    lab = np.zeros(yy.shape, np.int32)
    for k, (cx, cy) in enumerate(centers):
        d = (xx - cx) ** 2 + (yy - cy) ** 2
        m = d < best
        best[m], lab[m] = d[m], k
    labels = cv2.resize(lab.astype(np.float32), (s, s), interpolation=cv2.INTER_NEAREST).astype(int)
    palette = [(70, 120, 60), (110, 150, 80), (150, 170, 100), (190, 180, 120),
               (130, 110, 80), (90, 140, 70), (170, 160, 90), (100, 90, 70)]
    fine = noise(rng, s, 256, 4)
    for k in range(n_fields):
        m = labels == k
        col = np.array(palette[rng.integers(len(palette))], np.float32)
        # борозды: полосы под случайным углом с периодом 0,5--2 м
        ang = rng.uniform(0, math.pi)
        period = to_px(rng.uniform(0.5, 2.0)) + 1
        yi, xi = np.nonzero(m)
        stripes = 0.93 + 0.07 * np.sin(2 * math.pi * (xi * math.cos(ang) + yi * math.sin(ang)) / period)
        shade = (0.8 + 0.4 * fine[yi, xi]) * stripes
        img[yi, xi] = col * shade[:, None]
    # дороги
    for _ in range(10):
        p1 = tuple(int(v) for v in rng.integers(0, s, 2))
        p2 = tuple(int(v) for v in rng.integers(0, s, 2))
        cv2.line(img, p1, p2, (150, 150, 145), to_px(rng.uniform(4, 7)), cv2.LINE_AA)
    # лесополосы вдоль части границ полей и отдельные деревья
    edges = cv2.morphologyEx((labels % 7 == 0).astype(np.uint8), cv2.MORPH_GRADIENT, np.ones((25, 25), np.uint8))
    ys, xs = np.nonzero(edges[::40, ::40])
    for y, x in zip(ys * 40, xs * 40):
        r = to_px(rng.uniform(2, 4))
        cv2.circle(img, (int(x) + r // 3, int(y) + r // 3), r, (25, 35, 20), -1, cv2.LINE_AA)
        cv2.circle(img, (int(x), int(y)), r, (45, 80, 40), -1, cv2.LINE_AA)
    draw_trees(img, rng, 3000, (1.5, 4.0), [(40, 85, 40), (55, 95, 45), (35, 70, 35)])
    return img


def texture_forest(rng):
    """Сплошной лес: плотные кроны одного масштаба, повторяющийся рисунок."""
    s = TEX_PX
    base = noise(rng, s, 64, 5)
    img = np.stack([30 + 20 * base, 55 + 30 * base, 30 + 15 * base], -1).astype(np.float32)
    area = GROUND_SIZE ** 2
    draw_trees(img, rng, int(area / (math.pi * 3.0 ** 2) * 1.6), (2.0, 4.5),
               [(35, 75, 35), (45, 90, 40), (30, 60, 30), (55, 100, 50)])
    return img


def texture_field(rng):
    """Луг: низкий контраст, без дорог и деревьев. Признаки дают только пятна
    травы разной густоты, кочки и редкие проплешины."""
    s = TEX_PX
    low = noise(rng, s, 16, 4)
    mid = noise(rng, s, 160, 3)
    fine = noise(rng, s, 1024, 3)
    v = 0.82 + 0.16 * low + 0.14 * mid + 0.10 * fine
    img = np.stack([95 * v, 135 * v, 70 * v], -1).astype(np.float32)
    # кочки и проплешины: небольшие пятна слабого контраста
    for _ in range(40000):
        c = (int(rng.integers(0, s)), int(rng.integers(0, s)))
        r = to_px(rng.uniform(0.2, 0.8))
        k = rng.uniform(0.8, 1.2)
        col = (95 * k, 135 * k, 70 * k) if rng.random() < 0.85 else (120 * k, 120 * k, 85 * k)
        cv2.circle(img, c, max(r, 1), col, -1, cv2.LINE_AA)
    return img


TEXTURES = {"mixed": texture_mixed, "forest": texture_forest, "field": texture_field}


# ---------- модель ----------

def imu_noise_xml(nd, rw, turn_on_bias):
    """Белый шум с плотностью nd и постоянное смещение при включении.

    Блуждание смещения (dynamic_bias_*) не моделируется: при проверке реализация
    Gazebo давала уход смещения акселерометра ~0,4 м/с^2 за 30 с, в десятки раз
    больше, чем у ADIS16448; за время полёта смещение ADIS16448 меняется мало.
    Параметр rw сохраняется в calib.yaml для настройки VIO.
    """
    sigma = nd * math.sqrt(IMU_RATE)
    axes = ""
    for a in "xyz":
        axes += f"""<{a}><noise type="gaussian"><mean>0</mean><stddev>{sigma:.6g}</stddev>
              <bias_mean>{turn_on_bias:.6g}</bias_mean><bias_stddev>{turn_on_bias / 2:.6g}</bias_stddev></noise></{a}>
            """
    return axes


def make_model(out):
    src = (AG / "iris_with_ardupilot/model.sdf").read_text()
    src = src.replace('<model name="iris_with_ardupilot">', '<model name="iris_vio">')
    x, y, z, r, p, yw = CAM_POSE
    extra = f"""
    <!-- ===== датчики для визуально-инерциальной навигации ===== -->
    <link name="cam0_link">
      <pose relative_to="iris_with_standoffs::base_link">{x} {y} {z} {r} {p} {yw}</pose>
      <inertial><mass>0.005</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia></inertial>
      <sensor name="cam0" type="camera">
        <always_on>1</always_on>
        <update_rate>{CAM['rate']}</update_rate>
        <topic>cam0/image</topic>
        <camera>
          <horizontal_fov>{CAM['hfov']}</horizontal_fov>
          <image><width>{CAM['width']}</width><height>{CAM['height']}</height><format>L8</format></image>
          <clip><near>0.1</near><far>2000</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.007</stddev></noise>
        </camera>
      </sensor>
    </link>
    <joint name="cam0_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>cam0_link</child>
    </joint>

    <link name="imu0_link">
      <pose relative_to="iris_with_standoffs::base_link">0 0 0 0 0 0</pose>
      <inertial><mass>0.005</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia></inertial>
      <sensor name="imu0" type="imu">
        <always_on>1</always_on>
        <update_rate>{IMU_RATE}</update_rate>
        <topic>imu0</topic>
        <imu>
          <angular_velocity>
            {imu_noise_xml(GYRO_ND, GYRO_RW, 0.003)}
          </angular_velocity>
          <linear_acceleration>
            {imu_noise_xml(ACC_ND, ACC_RW, 0.05)}
          </linear_acceleration>
        </imu>
      </sensor>
    </link>
    <joint name="imu0_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>imu0_link</child>
    </joint>

    <!-- истинная поза корпуса (совпадает с системой координат imu0) -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>iris_vio</robot_base_frame>
      <odom_publish_frequency>{IMU_RATE}</odom_publish_frequency>
      <odom_topic>ground_truth</odom_topic>
      <dimensions>3</dimensions>
    </plugin>
  </model>"""
    src = re.sub(r"\s*</model>\s*</sdf>\s*$", extra + "\n</sdf>\n", src)
    d = out / "models/iris_vio"
    d.mkdir(parents=True, exist_ok=True)
    (d / "model.sdf").write_text(src)
    (d / "model.config").write_text("""<?xml version="1.0"?>
<model><name>iris_vio</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>Iris с ArduPilot, камерой вниз и IMU для VIO</description></model>
""")


# ---------- мир ----------

WORLD = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.6 0.6 0.6</ambient>
      <background>0.7 0.8 0.9</background>
      <shadows>true</shadows>
    </scene>

    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>
          <!-- земля «светится» текстурой: яркость кадра задаётся самой текстурой,
               а не подбором освещения; тени деревьев нарисованы в текстуре -->
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 1</specular>
            <emissive>1 1 1 1</emissive>
            <pbr><metal>
              <emissive_map>{texture}</emissive_map>
              <roughness>1.0</roughness>
              <metalness>0.0</metalness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <include>
      <uri>model://iris_vio</uri>
      <pose degrees="true">0 0 0.195 0 0 90</pose>
    </include>
  </world>
</sdf>
"""


def write_calib(out):
    fx = CAM["width"] / 2 / math.tan(CAM["hfov"] / 2)
    x, y, z, r, p, yw = CAM_POSE
    # поворот звена камеры (Rz*Ry*Rx), затем переход от оси взгляда +X звена
    # к оптической системе (z -- вперёд, x -- вправо, y -- вниз)
    def rot(rx, ry, rz):
        cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx
    R_link_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)  # столбцы: x,y,z оптической СК в СК звена
    T = np.eye(4)
    T[:3, :3] = rot(r, p, yw) @ R_link_opt
    T[:3, 3] = (x, y, z)
    lines = [
        "# Калибровка симулированных датчиков (генерируется make_scene.py)",
        f"cam0: {{width: {CAM['width']}, height: {CAM['height']}, rate_hz: {CAM['rate']},",
        f"       fx: {fx:.4f}, fy: {fx:.4f}, cx: {(CAM['width'] - 1) / 2:.1f}, cy: {(CAM['height'] - 1) / 2:.1f},",
        "       distortion: [0, 0, 0, 0]}",
        "# T_BS: поза оптической СК камеры в СК корпуса (= СК imu0), по строкам",
        "T_BS: [" + ", ".join(f"{v:.6f}" for v in T.ravel()) + "]",
        f"imu0: {{rate_hz: {IMU_RATE}, gyroscope_noise_density: {GYRO_ND}, gyroscope_random_walk: {GYRO_RW},",
        f"       accelerometer_noise_density: {ACC_ND}, accelerometer_random_walk: {ACC_RW}}}",
        "",
    ]
    (out / "calib.yaml").write_text("\n".join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("/sim/generated"))
    ap.add_argument("--surfaces", nargs="+", default=list(TEXTURES))
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    out = args.out
    (out / "textures").mkdir(parents=True, exist_ok=True)
    (out / "worlds").mkdir(parents=True, exist_ok=True)
    make_model(out)
    write_calib(out)
    for name in args.surfaces:
        tex = out / f"textures/{name}.png"
        if not tex.exists():
            img = TEXTURES[name](np.random.default_rng(args.seed))
            # Gazebo считает текстуру закодированной в sRGB и переводит в линейную
            # яркость, а одноканальный кадр камеры отдаёт без обратного перевода;
            # заранее применяем обратную гамму, чтобы яркость в кадре совпала с задуманной
            # замена автоэкспозиции камеры: средняя яркость любой поверхности ~100
            img = img * (TARGET_MEAN / img.mean())
            img = 255.0 * (np.clip(img, 0, 255) / 255.0) ** (1 / 2.2)
            cv2.imwrite(str(tex), img[..., ::-1].astype(np.uint8))  # RGB -> BGR
            print(f"текстура {tex}")
        (out / f"worlds/{name}.sdf").write_text(
            WORLD.format(name=name, size=GROUND_SIZE, texture=f"/sim/generated/textures/{name}.png"))
        print(f"мир {out}/worlds/{name}.sdf")


if __name__ == "__main__":
    main()
