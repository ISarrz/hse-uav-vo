#!/usr/bin/env bash
# Запуск одного полёта внутри контейнера uav-sim.
# Использование: run_inside.sh SURFACE ALT SPEED [аргументы fly_mission.py]
# Результат: /out/mav0 (формат EuRoC) и логи в /out.
set -uo pipefail
SURFACE="$1"; ALT="$2"; SPEED="$3"; shift 3
export GZ_IP=127.0.0.1 GZ_PARTITION="sim$$"
export GZ_SIM_RESOURCE_PATH="/sim/generated/models:$GZ_SIM_RESOURCE_PATH"
OUT=/out
mkdir -p "$OUT"
FLAG="$OUT/record.flag"; rm -f "$FLAG"

gz sim -s -r --headless-rendering -v 3 "/sim/generated/worlds/$SURFACE.sdf" > "$OUT/gz.log" 2>&1 &
GZ=$!
sleep 8
python3 /sim/record_gz.py "$OUT/mav0" --start-file "$FLAG" > "$OUT/record.log" 2>&1 &
REC=$!

mkdir -p /tmp/sitl && cd /tmp/sitl
P=/opt/ardupilot/Tools/autotest/default_params
arducopter -S --model JSON --speedup 1 --slave 0 -I0 \
    --home -35.363262,149.165237,584,90 \
    --defaults "$P/copter.parm,$P/gazebo-iris.parm" > "$OUT/sitl.log" 2>&1 &
SITL=$!

python3 /sim/fly_mission.py --alt "$ALT" --speed "$SPEED" --flag "$FLAG" "$@" 2>&1 | tee "$OUT/mission.log"
STATUS=${PIPESTATUS[0]}

rm -f "$FLAG"
kill -INT $REC; wait $REC
# gz sim запускает сервер отдельными процессами и не всегда завершается по SIGTERM
kill $SITL $GZ 2>/dev/null
sleep 3
pkill -9 -f arducopter 2>/dev/null; pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f gz-sim 2>/dev/null
tail -1 "$OUT/record.log"
exit $STATUS
