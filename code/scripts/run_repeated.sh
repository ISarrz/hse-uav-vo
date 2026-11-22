#!/usr/bin/env bash
# Многократный запуск одной системы на последовательности.
# Использование: scripts/run_repeated.sh <orbslam3|vins|openvins> SEQ MODE N
# Каждый запуск сохраняется в results/SEQ/runs/<траектория>_run<k>.tum (+ логи),
# затем: scripts/evaluate.py SEQ --runs [--gnss-loss]
set -euo pipefail
SYS="$1"; SEQ="$2"; MODE="$3"; N="${4:-5}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$HERE/results/$SEQ"
RUNS="$OUT/runs"
mkdir -p "$RUNS"

case "$SYS" in
    orbslam3) PREFIX="orbslam3_$MODE" ;;
    vins)     PREFIX="vins_$MODE" ;;
    openvins) PREFIX="openvins_$MODE" ;;
    *) echo "неизвестная система $SYS"; exit 1 ;;
esac

for k in $(seq 1 "$N"); do
    echo "[$(date +%T)] $SYS $SEQ $MODE: запуск $k/$N"
    "$HERE/scripts/run_$SYS.sh" "$SEQ" "$MODE" | tail -1 || echo "  запуск $k завершился с ошибкой"
    for f in "$OUT/$PREFIX"*.tum "$OUT/$PREFIX"*.log; do
        [ -f "$f" ] || continue
        base=$(basename "$f"); ext="${base##*.}"; stem="${base%.*}"
        cp "$f" "$RUNS/${stem}_run$k.$ext"
    done
done
