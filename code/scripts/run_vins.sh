#!/usr/bin/env bash
# Запуск VINS-Fusion (с модулем замыкания петель) на последовательности EuRoC.
# Использование: scripts/run_vins.sh MH_01_easy [mono_imu|stereo_imu|stereo]
# Требуется bag: scripts/euroc_to_bag.py data/euroc/<SEQ> --ros1
# DATASET=sim CONFIG=sim -- данные data/sim/<SEQ>, конфигурация configs/sim/vins/
set -euo pipefail
SEQ="$1"; MODE="${2:-mono_imu}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
SEQ_DIR="$HERE/data/${DATASET:-euroc}/$SEQ"
CONFIG="${CONFIG:-}"
CFG_MOUNT=(); CFG_SRC=/opt/catkin_ws/src/VINS-Fusion/config/euroc; CFG_NAME="euroc_${MODE}_config.yaml"
if [ -n "$CONFIG" ]; then
    CFG_MOUNT=(-v "$HERE/configs/$CONFIG/vins":/cfg:ro,Z); CFG_SRC=/cfg; CFG_NAME="${MODE}_config.yaml"
fi
OUT="$HERE/results/$SEQ"
mkdir -p "$OUT"
rm -f "$OUT/vio.csv" "$OUT/vio_loop.csv" "$OUT/vins_online.bag"

# сеть хоста нужна roscore; ROS_MASTER_URI на отдельном порту, чтобы не мешать другим запускам
podman run --rm --network=host -e ROS_MASTER_URI=http://localhost:11411 \
    -v "$SEQ_DIR":/data:ro,Z -v "$OUT":/work:Z "${CFG_MOUNT[@]}" -w /work \
    localhost/uav-vins bash -c "
        source /opt/catkin_ws/devel/setup.bash
        cp -r $CFG_SRC /tmp/cfg
        CFG=/tmp/cfg/$CFG_NAME
        sed -i 's|^output_path:.*|output_path: \"/work/\"|' \$CFG
        roscore -p 11411 > /dev/null 2>&1 &
        sleep 3
        rosrun vins vins_node \$CFG > /work/vins_${MODE}.log 2>&1 &
        rosrun loop_fusion loop_fusion_node \$CFG > /work/vins_${MODE}_loop.log 2>&1 &
        sleep 3
        # онлайн-оценка с учётом петель: поза VIO, исправленная текущим графом поз
        rosbag record -q -O /work/vins_online.bag -e '.*odometry_rect' __name:=online_rec > /dev/null 2>&1 &
        sleep 2
        rosbag play -q /data/$SEQ.bag
        sleep 5
        rosnode kill /online_rec > /dev/null 2>&1; sleep 2
        kill -INT %4 %3 %2 %1 2>/dev/null; sleep 3
    "

# VINS пишет: t[нс], px, py, pz, qw, qx, qy, qz, ...  ->  TUM: t[с] px py pz qx qy qz qw
to_tum() { awk -F, 'NF>=8 { printf "%.9f %s %s %s %s %s %s %s\n", $1/1e9, $2, $3, $4, $6, $7, $8, $5 }' "$1" > "$2"; }
to_tum "$OUT/vio.csv" "$OUT/vins_$MODE.tum"
to_tum "$OUT/vio_loop.csv" "$OUT/vins_${MODE}_loop.tum"
"$HERE/.venv/bin/python" -I "$HERE/scripts/odom_bag_to_tum.py" "$OUT/vins_online.bag" "$OUT/vins_${MODE}_loop_online.tum"
echo "$SEQ vins_$MODE: $(wc -l < "$OUT/vins_$MODE.tum") поз VIO, $(wc -l < "$OUT/vins_${MODE}_loop.tum") поз после замыкания петель"
