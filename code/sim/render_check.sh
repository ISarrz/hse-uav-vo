#!/usr/bin/env bash
# Проверка картинки камеры без полёта: коптер появляется на высоте ALT и падает,
# записываются первые секунды. Использование (в контейнере): render_check.sh SURFACE ALT
set -uo pipefail
SURFACE="$1"; ALT="${2:-30}"
export GZ_IP=127.0.0.1 GZ_PARTITION="chk$$"
export GZ_SIM_RESOURCE_PATH="/sim/generated/models:$GZ_SIM_RESOURCE_PATH"
W=/tmp/check.sdf
sed "s|<pose degrees=\"true\">0 0 0.195 0 0 90</pose>|<pose degrees=\"true\">0 0 $ALT 0 0 90</pose>|" \
    "/sim/generated/worlds/$SURFACE.sdf" > "$W"
python3 /sim/record_gz.py /out/check > /dev/null 2>&1 &
REC=$!
timeout 25 gz sim -s -r --headless-rendering "$W" > /out/check_gz.log 2>&1
kill -INT $REC; wait $REC
pkill -9 -f gz-sim 2>/dev/null
ls /out/check/cam0/data | wc -l
