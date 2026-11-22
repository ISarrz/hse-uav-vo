#!/usr/bin/env python3
"""Полёт коптера в SITL по маршруту поисковой миссии (змейка) через MAVLink.

    python3 fly_mission.py --alt 40 --speed 5 --width 200 --length 200 --spacing 50 \
        --flag /out/record.flag

Порядок: ожидание готовности, взлёт (режим GUIDED), разгонный манёвр для
инициализации VIO, облёт змейкой на высоте --alt со скоростью --speed,
возврат к точке старта и посадка. Файл --flag существует, пока идёт запись данных:
от висения на заданной высоте до начала посадки.
"""
import argparse
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

M = mavutil.mavlink


def log(*a):
    print(f"[{time.strftime('%H:%M:%S')}]", *a, flush=True)


class Vehicle:
    def __init__(self, url):
        self.m = mavutil.mavlink_connection(url, source_system=250)
        self.m.wait_heartbeat(timeout=120)
        log("связь с автопилотом установлена")
        self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component,
                                            M.MAV_DATA_STREAM_ALL, 10, 1)

    def param(self, name, value):
        """Установка параметра с проверкой: автопилот отвечает PARAM_VALUE."""
        for _ in range(3):
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(),
                                      float(value), M.MAV_PARAM_TYPE_REAL32)
            t0 = time.time()
            while time.time() - t0 < 2:
                msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
                if msg and msg.param_id == name and abs(msg.param_value - value) < 1e-3:
                    return True
        log(f"параметр {name} не установлен")
        return False

    def mode(self, name):
        mode_id = self.m.mode_mapping()[name]
        while True:
            self.m.set_mode(mode_id)
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
            if hb and hb.custom_mode == mode_id:
                return

    def cmd(self, command, *params):
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, command, 0, *p)
        t0 = time.time()
        while time.time() - t0 < 5:
            msg = self.m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                log("автопилот:", msg.text)
            elif msg.command == command:
                if msg.result != M.MAV_RESULT_ACCEPTED:
                    log(f"команда {command} отклонена (код {msg.result})")
                return msg.result == M.MAV_RESULT_ACCEPTED
        log(f"нет ответа на команду {command}")
        return False

    def pos(self):
        """Положение в локальной СК NED относительно точки старта."""
        while True:
            msg = self.m.recv_match(type=["LOCAL_POSITION_NED", "STATUSTEXT"], blocking=True, timeout=5)
            if msg is None:
                return None
            if msg.get_type() == "STATUSTEXT":
                log("автопилот:", msg.text)
                continue
            return msg.x, msg.y, msg.z

    def arm_when_ready(self, timeout=300):
        t0 = time.time()
        while time.time() - t0 < timeout:
            # дожидаемся сходимости EKF и GPS: пробуем включить моторы, пока не получится
            self.mode("GUIDED")
            if self.cmd(M.MAV_CMD_COMPONENT_ARM_DISARM, 1):
                log("моторы включены")
                return
            time.sleep(3)
        raise SystemExit("автопилот не готов к взлёту")

    def goto(self, n, e, d, speed, tol=1.0, timeout=600):
        """Полёт в точку (NED) с заданной горизонтальной скоростью."""
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component, M.MAV_FRAME_LOCAL_NED,
            0b0000111111111000, n, e, d, 0, 0, 0, 0, 0, 0, 0, 0)
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self.pos()
            if p and math.dist(p, (n, e, d)) < tol:
                return
        raise SystemExit(f"не долетел до точки {n:.0f} {e:.0f} {d:.0f}")


def lawnmower(width, length, spacing):
    """Змейка: проходы вдоль оси север, смещение между проходами -- на восток."""
    pts, e, k = [], -width / 2, 0
    while e <= width / 2 + 1e-6:
        n0, n1 = (0, length) if k % 2 == 0 else (length, 0)
        pts += [(n0, e), (n1, e)]
        e += spacing
        k += 1
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--alt", type=float, default=40.0, help="высота полёта, м")
    ap.add_argument("--speed", type=float, default=5.0, help="горизонтальная скорость, м/с")
    ap.add_argument("--width", type=float, default=200.0)
    ap.add_argument("--length", type=float, default=200.0)
    ap.add_argument("--spacing", type=float, default=50.0, help="расстояние между проходами, м")
    ap.add_argument("--hover", type=float, default=5.0, help="висение перед началом движения, с")
    ap.add_argument("--flag", type=Path)
    args = ap.parse_args()

    v = Vehicle(args.url)
    # ArduPilot 4.7: параметры в м/с и м/с^2 (раньше WPNAV_SPEED и др. в см/с)
    if not v.param("WP_SPD", args.speed):
        sys.exit("не удалось задать скорость полёта")
    v.param("WP_ACC", 2.5)
    v.param("WP_SPD_UP", 3.0)
    v.param("WP_SPD_DN", 2.0)
    v.param("WP_YAW_BEHAVIOR", 1)  # нос по направлению полёта
    v.arm_when_ready()

    log(f"взлёт на {args.alt} м")
    while not v.cmd(M.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.alt):
        # моторы могли выключиться по таймауту, пока команда отклонялась
        time.sleep(2)
        v.arm_when_ready()
    while True:
        p = v.pos()
        if p and -p[2] > args.alt - 1.0:
            break

    # запись начинается в воздухе: у земли камера в 15 см от поверхности, картинка
    # вырождена. Несколько секунд висения нужны для статической инициализации VIO.
    v.goto(0, 0, -args.alt, args.speed, tol=0.5)
    if args.flag:
        args.flag.touch()
    t_start = time.time()
    log(f"висение {args.hover} с")
    t0 = time.time()
    while time.time() - t0 < args.hover:
        v.pos()

    # разгонный манёвр: небольшой квадрат с изменением высоты -- ускорения по всем осям
    log("разгонный манёвр")
    d = -args.alt
    for n, e, dz in ((15, 0, -3), (15, 15, 3), (0, 15, -3), (0, 0, 0)):
        v.goto(n, e, d + dz, args.speed)

    # скорость дополнительно задаётся командой: в GUIDED она действует до смены режима
    v.cmd(M.MAV_CMD_DO_CHANGE_SPEED, 1, args.speed, -1)
    route = lawnmower(args.width, args.length, args.spacing)
    log(f"облёт: {len(route)} точек, высота {args.alt} м, скорость {args.speed} м/с")
    for i, (n, e) in enumerate(route):
        v.goto(n, e, d, args.speed, tol=2.0)
        log(f"точка {i + 1}/{len(route)}")
    v.goto(0, 0, d, args.speed, tol=2.0)

    if args.flag and args.flag.exists():
        args.flag.unlink()
    log(f"полёт завершён за {time.time() - t_start:.0f} с, посадка")
    v.mode("LAND")
    t0 = time.time()
    while time.time() - t0 < 300:
        hb = v.m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if hb and not hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED:
            log("посадка выполнена")
            return
    sys.exit("посадка не завершилась")


if __name__ == "__main__":
    main()
