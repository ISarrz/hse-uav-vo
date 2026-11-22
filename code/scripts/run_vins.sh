#!/usr/bin/env bash
# Запуск VINS-Fusion (с модулем замыкания петель) на последовательности EuRoC.
# Использование: scripts/run_vins.sh MH_01_easy [mono_imu|stereo_imu|stereo]
# Требуется bag: scripts/euroc_to_bag.py data/euroc/<SEQ> --ros1
set -euo pipefail
SEQ="$1"; MODE="${2:-mono_imu}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
SEQ_DIR="$HERE/data/euroc/$SEQ"
OUT="$HERE/results/$SEQ"
mkdir -p "$OUT"
rm -f "$OUT/vio.csv" "$OUT/vio_loop.csv"

# сеть хоста нужна roscore; ROS_MASTER_URI на отдельном порту, чтобы не мешать другим запускам
podman run --rm --network=host -e ROS_MASTER_URI=http://localhost:11411 \
    -v "$SEQ_DIR":/data:ro,Z -v "$OUT":/work:Z -w /work \
    localhost/uav-vins bash -c "
        source /opt/catkin_ws/devel/setup.bash
        cp -r /opt/catkin_ws/src/VINS-Fusion/config/euroc /tmp/cfg
        CFG=/tmp/cfg/euroc_${MODE}_config.yaml
        sed -i 's|^output_path:.*|output_path: \"/work/\"|' \$CFG
        roscore -p 11411 > /dev/null 2>&1 &
        sleep 3
        rosrun vins vins_node \$CFG > /work/vins_${MODE}.log 2>&1 &
        rosrun loop_fusion loop_fusion_node \$CFG > /work/vins_${MODE}_loop.log 2>&1 &
        sleep 3
        rosbag play -q /data/$SEQ.bag
        sleep 5
        kill -INT %3 %2 %1 2>/dev/null; sleep 3
    "

# VINS пишет: t[нс], px, py, pz, qw, qx, qy, qz, ...  ->  TUM: t[с] px py pz qx qy qz qw
to_tum() { awk -F, 'NF>=8 { printf "%.9f %s %s %s %s %s %s %s\n", $1/1e9, $2, $3, $4, $6, $7, $8, $5 }' "$1" > "$2"; }
to_tum "$OUT/vio.csv" "$OUT/vins_$MODE.tum"
to_tum "$OUT/vio_loop.csv" "$OUT/vins_${MODE}_loop.tum"
echo "$SEQ vins_$MODE: $(wc -l < "$OUT/vins_$MODE.tum") поз VIO, $(wc -l < "$OUT/vins_${MODE}_loop.tum") поз после замыкания петель"
