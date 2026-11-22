#!/usr/bin/env bash
# Сборка контейнеров с ORB-SLAM3, VINS-Fusion, OpenVINS и симулятором (sim).
# Использование: scripts/build_images.sh [orbslam3|vins|openvins|sim ...]
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
CTX="$HERE/containers"
LOGS="$HERE/logs"
mkdir -p "$LOGS"

# --network=host: в rootless podman сеть pasta ломается при большом числе маршрутов (VPN)
targets=("${@:-openvins vins orbslam3}")
for t in ${targets[@]}; do
    echo "[$(date +%T)] building uav-$t"
    if podman build --network=host -t "localhost/uav-$t" -f "$CTX/$t.Containerfile" "$CTX" > "$LOGS/build-$t.log" 2>&1; then
        echo "[$(date +%T)] uav-$t OK"
    else
        echo "[$(date +%T)] uav-$t FAILED, see logs/build-$t.log"
    fi
done
