#!/usr/bin/env python3
"""Проверка логики LTC-декодера на тестовом WAV.

Воспроизводит цепочку скетча: переходы через ноль -> интервалы ->
bi-phase mark (короткий/длинный) -> детектор sync word -> кадр 80 бит ->
разбор BCD по стандарту SMPTE 12M (LSB-first; user bits вклиниваются между
нибблами — tens каждого поля лежит в СЛЕДУЮЩЕМ байте, а не в старшем ниббле).

Проверяет, что декодированный таймкод идёт монотонно (+1 кадр).

Запуск: python3 tools/ltc_wav_check.py [файл.wav] [бит/с]
  Если бит/с не задан — перебираются 2000/1920/2400 (25/24/30 fps).
"""
import sys
import wave
import struct

path = sys.argv[1] if len(sys.argv) > 1 else \
    'LTC_01000000_5mins_25_FPS_48000x16.wav'
bitrates = [int(sys.argv[2])] if len(sys.argv) > 2 else [2000, 1920, 2400]

w = wave.open(path)
sr = w.getframerate()
ch = w.getnchannels()
sw = w.getsampwidth()
n = w.getnframes()
print('%s: %d Hz, %d ch, %d bit, %.1f s' % (path, sr, ch, sw * 8, n / float(sr)))
if sw != 2:
    sys.exit('ожидался 16-битный WAV')

raw = w.readframes(min(sr * 5, n))                # первые ~5 секунд
all_s = struct.unpack('<%dh' % (len(raw) // sw), raw)
s = all_s[0::ch]                                  # канал 0

# --- переходы через ноль (аналог компаратора) ---
edges = []
prev = 0
for i, v in enumerate(s):
    cur = 1 if v >= 0 else -1
    if prev != 0 and cur != prev:
        edges.append(i)
    prev = cur
print('переходов: %d' % len(edges))

MIN_EDGE = sr * 80e-6                             # антидребезг 80 мкс
SYNC = [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1]


def demod(bits_per_sec):
    """Интервалы -> биты (bi-phase mark) -> кадры 80 бит по границе sync."""
    BIT = sr / float(bits_per_sec)
    thr = 0.75 * BIT
    bits = []
    half = False
    last = edges[0]
    for e in edges[1:]:
        d = e - last
        last = e
        if d < MIN_EDGE:
            continue                              # дребезг
        if d < thr:
            if half:
                half = False
                bits.append(1)                    # пара коротких -> 1
            else:
                half = True
        else:
            half = False
            bits.append(0)                        # длинный -> 0
    frames = []
    i = 0
    while i <= len(bits) - 16:
        if bits[i:i + 16] == SYNC:
            if i >= 64:
                frames.append(bits[i - 64:i + 16])  # полный кадр 80 бит
            i += 16
        else:
            i += 1
    return len(bits), frames


frames = []
bits_total = 0
for br in bitrates:
    bits_total, frames = demod(br)
    if frames:
        print('битовая скорость: %d бит/с, бит %d, sync-кадров %d'
              % (br, bits_total, len(frames)))
        break
if not frames:
    sys.exit('sync word не найден — проверить сигнал/пороги')

# Раскладка SMPTE 12M (LSB-first), tens каждого поля — в СЛЕДУЮЩЕМ байте:
#  0-3 кадры ед. | 8-9 кадры дес. | 10 DF
# 16-19 сек ед.  | 24-26 сек дес.
# 32-35 мин ед.  | 40-42 мин дес.
# 48-51 час ед.  | 56-57 час дес.
def field(b, start, width):
    return sum(b[start + k] << k for k in range(width))


def parse(f):
    return dict(
        hu=field(f, 48, 4), ht=field(f, 56, 2),
        mu=field(f, 32, 4), mt=field(f, 40, 3),
        su=field(f, 16, 4), st=field(f, 24, 3),
        fu=field(f, 0, 4), ft=field(f, 8, 2),
        df=field(f, 10, 1),
    )


def fmt(p):
    return '%02d:%02d:%02d:%02d%s' % (
        p['ht'] * 10 + p['hu'], p['mt'] * 10 + p['mu'],
        p['st'] * 10 + p['su'], p['ft'] * 10 + p['fu'],
        ' DF' if p['df'] else '')


def bytes_of(f):
    return [field(f, 8 * j, 8) for j in range(10)]


print('\nпервый кадр: %s  bytes=%s'
      % (fmt(parse(frames[0])), ' '.join('%02X' % x for x in bytes_of(frames[0]))))

# --- монотонность: следующий кадр = +1 либо начало секунды (0) ---
gaps = 0
prev_f = None
for f in frames:
    cur = field(f, 0, 4) + field(f, 8, 2) * 10
    if prev_f is not None and cur != 0 and cur != prev_f + 1:
        gaps += 1
    prev_f = cur

last = parse(frames[-1])
maxf = max(field(f, 0, 4) + field(f, 8, 2) * 10 for f in frames)
print('кадров: %d, разрывов последовательности: %d' % (len(frames), gaps))
print('последний: %s  fps≈%s' % (fmt(last), {23: 24, 24: 25, 29: 30}.get(maxf, '?')))
