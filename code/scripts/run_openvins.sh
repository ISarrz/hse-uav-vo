#!/usr/bin/env bash
# Запуск OpenVINS на последовательности EuRoC.
# Использование: scripts/run_openvins.sh MH_01_easy [mono|stereo]
# Требуется bag: scripts/euroc_to_bag.py data/euroc/<SEQ> --ros2
set -euo pipefail
SEQ="$1"; MODE="${2:-mono}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
SEQ_DIR="$HERE/data/euroc/$SEQ"
OUT="$HERE/results/$SEQ"
mkdir -p "$OUT"
if [ "$MODE" = mono ]; then STEREO=false; NCAM=1; else STEREO=true; NCAM=2; fi

# ROS_LOCALHOST_ONLY и отдельный домен, чтобы DDS не выходил в сеть хоста
podman run --rm --network=host -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=$((RANDOM % 100 + 100)) \
    -v "$SEQ_DIR":/data:ro,Z -v "$OUT":/work:Z -w /work \
    localhost/uav-openvins bash -c "
        source /opt/ws/install/setup.bash
        cp -r /opt/ws/src/open_vins/config/euroc_mav /tmp/cfg
        CFG=/tmp/cfg/estimator_config.yaml
        sed -i -e 's|^use_stereo:.*|use_stereo: $STEREO|' -e 's|^max_cameras:.*|max_cameras: $NCAM|' \
               -e 's|^save_total_state:.*|save_total_state: true|' \
               -e 's|^filepath_est:.*|filepath_est: \"/work/openvins_${MODE}_state.txt\"|' \
               -e 's|^filepath_std:.*|filepath_std: \"/work/openvins_${MODE}_std.txt\"|' \
               -e 's|^record_timing_information:.*|record_timing_information: true|' \
               -e 's|^record_timing_filepath:.*|record_timing_filepath: \"/work/openvins_${MODE}_timing.txt\"|' \$CFG
        # параметры сохранения читаются из параметров узла ROS 2, а не из YAML
        ros2 run ov_msckf run_subscribe_msckf --ros-args -p config_path:=\$CFG \
            -p save_total_state:=true \
            -p filepath_est:=/work/openvins_${MODE}_state.txt \
            -p filepath_std:=/work/openvins_${MODE}_std.txt > /work/openvins_${MODE}.log 2>&1 &
        sleep 5
        ros2 bag play /data/${SEQ}_ros2 > /dev/null 2>&1
        sleep 5
        kill -INT %1 2>/dev/null; sleep 3
    "

# OpenVINS: t[с] q_GtoI(x,y,z,w, JPL) p_IinG ...; компоненты JPL q_GtoI совпадают
# с гамильтоновым кватернионом поворота IMU->мир, поэтому переставляем только столбцы
awk '!/^#/ && NF>=8 { printf "%.9f %s %s %s %s %s %s %s\n", $1, $6, $7, $8, $2, $3, $4, $5 }' \
    "$OUT/openvins_${MODE}_state.txt" > "$OUT/openvins_$MODE.tum"
echo "$SEQ openvins_$MODE: $(wc -l < "$OUT/openvins_$MODE.tum") поз"
