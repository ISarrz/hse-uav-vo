#!/usr/bin/env bash
# Один полёт в симуляции (ArduPilot SITL + Gazebo) с записью данных в формате EuRoC.
# Использование: scripts/run_sim.sh NAME SURFACE ALT SPEED [аргументы fly_mission.py]
#   SURFACE: mixed | forest | field (см. sim/make_scene.py)
# Результат: data/sim/NAME/mav0 -- дальше как с EuRoC:
#   .venv/bin/python -I scripts/euroc_to_bag.py data/sim/NAME --ros1 --ros2
set -euo pipefail
NAME="$1"; SURFACE="$2"; ALT="$3"; SPEED="$4"; shift 4
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$HERE/data/sim/$NAME"
if [ -d "$OUT/mav0" ]; then echo "$OUT уже существует"; exit 1; fi
mkdir -p "$OUT"

# сцена генерируется один раз
if [ ! -f "$HERE/sim/generated/worlds/$SURFACE.sdf" ]; then
    podman run --rm --network=none -v "$HERE/sim":/sim:Z localhost/uav-sim \
        python3 /sim/make_scene.py --out /sim/generated
fi

# /dev/dri -- рендеринг камеры на встроенной графике
podman run --rm --network=none --device /dev/dri \
    -v "$HERE/sim":/sim:Z -v "$OUT":/out:Z localhost/uav-sim \
    bash /sim/run_inside.sh "$SURFACE" "$ALT" "$SPEED" "$@"
printf 'surface: %s\nalt_m: %s\nspeed_mps: %s\nextra: "%s"\n' "$SURFACE" "$ALT" "$SPEED" "$*" > "$OUT/params.yaml"
