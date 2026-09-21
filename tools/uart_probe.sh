#!/usr/bin/env bash
# Короткий захват потока UART с Nano (одна попытка, не мучить порт).
# Использование: uart_probe.sh [секунды] [файл_вывода]
set -u
PORT=${LTC_PORT:-/dev/cu.usbserial-A5069RR4}
SECS=${1:-4}
OUT=${2:-/tmp/ltc_uart_capture.txt}

if [ ! -c "$PORT" ]; then
  echo "ПОРТ НЕ НАЙДЕН: $PORT (плата подключена к USB?)"
  exit 1
fi

stty -f "$PORT" 115200 raw || exit 1
cat "$PORT" > "$OUT" &
PID=$!
sleep "$SECS"
kill "$PID" 2>/dev/null
wait "$PID" 2>/dev/null

BYTES=$(wc -c < "$OUT" | tr -d ' ')
echo "Захвачено $BYTES байт за ${SECS}с → $OUT"
# Ожидание при здоровом канале: ~600-900 байт/с (D-строка ~90 Б/с + TC-строки 24-30/с ~ 12 Б/с).
