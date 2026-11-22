#!/usr/bin/env bash
# Запуск ORB-SLAM3 на последовательности EuRoC.
# Использование: scripts/run_orbslam3.sh MH_01_easy [mono_inertial|stereo_inertial|mono|stereo]
# DATASET=sim CONFIG=sim scripts/run_orbslam3.sh <полёт> mono_inertial  -- данные data/sim/<полёт>,
#   конфигурация configs/sim/orbslam3_<режим>.yaml (scripts/make_sim_configs.py)
set -euo pipefail
SEQ="$1"; MODE="${2:-mono_inertial}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
DATA="$HERE/data/${DATASET:-euroc}"
CONFIG="${CONFIG:-}"
OUT="$HERE/results/$SEQ"
mkdir -p "$OUT"

# MH_01_easy -> MH01, V1_02_medium -> V102
TS=$(echo "$SEQ" | sed -E 's/^(MH|V[12])_([0-9]{2}).*/\1\2/')
case "$MODE" in
    mono_inertial)   DIR=Monocular-Inertial; BIN=mono_inertial_euroc;   CFG=EuRoC.yaml ;;
    stereo_inertial) DIR=Stereo-Inertial;    BIN=stereo_inertial_euroc; CFG=EuRoC.yaml ;;
    mono)            DIR=Monocular;          BIN=mono_euroc;            CFG=EuRoC.yaml ;;
    stereo)          DIR=Stereo;             BIN=stereo_euroc;          CFG=EuRoC.yaml ;;
    *) echo "неизвестный режим $MODE"; exit 1 ;;
esac

# своя конфигурация и список кадров для данных не из EuRoC
CFG_MOUNT=(); CFG_PATH="\$E/$CFG"; TS_PATH="\$E/EuRoC_TimeStamps/$TS.txt"
if [ -n "$CONFIG" ]; then
    CFG_MOUNT=(-v "$HERE/configs/$CONFIG":/cfg:ro,Z); CFG_PATH="/cfg/orbslam3_$MODE.yaml"
fi
if [ "${DATASET:-euroc}" != euroc ]; then
    grep -v '^#' "$DATA/$SEQ/mav0/cam0/data.csv" | cut -d, -f1 > "$OUT/orbslam3_timestamps.txt"
    TS_PATH=/work/orbslam3_timestamps.txt
fi

podman run --rm --network=none \
    -v "$DATA/$SEQ":/data:ro,Z -v "$OUT":/work:Z "${CFG_MOUNT[@]}" -w /work \
    localhost/uav-orbslam3 bash -c "
        E=/opt/ORB_SLAM3/Examples/$DIR
        cp $CFG_PATH /work/orbslam3_config.yaml
        rm -f /work/online_trajectory.txt
        \$E/$BIN /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt $CFG_PATH /data $TS_PATH orbslam3_$MODE
    " > "$OUT/orbslam3_$MODE.log" 2>&1

# ORB-SLAM3 пишет время в наносекундах; для evo переводим в секунды (формат TUM)
SRC="$OUT/f_orbslam3_$MODE.txt"
[ "$MODE" = mono ] && SRC="$OUT/kf_orbslam3_$MODE.txt"
awk '{ printf "%.9f %s %s %s %s %s %s %s\n", $1/1e9, $2, $3, $4, $5, $6, $7, $8 }' "$SRC" > "$OUT/orbslam3_$MODE.tum"
echo "$SEQ orbslam3_$MODE: $(wc -l < "$OUT/orbslam3_$MODE.tum") поз -> results/$SEQ/orbslam3_$MODE.tum"

# онлайн-оценка (поза в момент кадра, без последующих уточнений) -- только для режимов с IMU
if [ -f "$OUT/online_trajectory.txt" ]; then
    "$HERE/.venv/bin/python" -I "$HERE/scripts/orbslam3_online_to_tum.py" \
        "$OUT/online_trajectory.txt" "$OUT/orbslam3_config.yaml" "$OUT/orbslam3_${MODE}_online.tum"
    mv "$OUT/online_trajectory.txt" "$OUT/orbslam3_${MODE}_online_raw.txt"
fi
