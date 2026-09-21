#!/usr/bin/env python3
"""Анализ захваченного UART-потока LTC reader.

Читает файл захвата (tools/probe_reset.py), разделяет строки на:
  - D-строки диагностики:
      D E=<n> deb=<n> S=<n> L=<n> sync=<n> rej=<n> flt=<n> gap=<n> thr=<n> raw=<hex>
      E   — фронтов вычитано из буфера за секунду
      deb — отброшено антидребезгом
      S/L — короткие/длинные интервалы
      sync— совпадений sync word
      rej — кадров sync найдено, но parseFrame отверг (BCD/диапазон → сбой)
      flt — отклонено фильтром правдоподобия (валидный, но чужой TC)
      gap — сбросов фазы декодера (разрыв сигнала)
      thr — адаптивный порог, тики
  - TC-строки: TC HH:MM:SS:FF [DF] fps=<n>
  - мусор (обрезанное/слипшееся/нечисл. поля)

Эталон 24 fps (файлы LTC_0200..., ~23% единиц): E≈2365, deb≈0, S≈886,
L≈1479, sync≈24, rej≈0, flt≈0, gap≈0, thr≈781-790.

Использование: python3 analyze_capture.py [файл] (по умолчанию /tmp/ltc_uart_capture.txt)
"""
import re
import statistics
import sys

D_RE = re.compile(
    r"D E=(\d+) deb=(\d+) S=(\d+) L=(\d+) sync=(\d+) "
    r"rej=(\d+) flt=(\d+) gap=(\d+) thr=(\d+) raw=([0-9A-Fa-f]{20})"
)
TC_RE = re.compile(
    r"TC (\d{2}):(\d{2}):(\d{2}):(\d{2})( DF)? fps=(\d+)"
)
D_HINT = re.compile(r"^\s*(D|d)\b")
TC_HINT = re.compile(r"^TC\b")

NAMES = ["E", "deb", "S", "L", "sync", "rej", "flt", "gap", "thr"]
# Эталоны (24 fps, здоровый вход, реальные файлы)
REF = {"E": 2365, "deb": 0, "S": 886, "L": 1479, "sync": 24,
       "rej": 0, "flt": 0, "gap": 0}


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/ltc_uart_capture.txt"
    try:
        data = open(path, "rb").read()
    except OSError as e:
        print(f"не читается {path}: {e}")
        return 1
    print(f"файл: {path}, {len(data)} байт")

    text = data.decode("ascii", errors="replace")
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]

    d_ok, d_bad, tc_ok, tc_bad, other = [], [], [], [], []
    for ln in lines:
        if D_HINT.match(ln):
            m = D_RE.match(ln)
            (d_ok if m else d_bad).append(m.groups() if m else ln)
        elif TC_HINT.search(ln):
            m = TC_RE.search(ln)
            (tc_ok if m else tc_bad).append(m.groups() if m else ln)
        else:
            other.append(ln)

    print(
        f"строк: всего {len(lines)}, D ok {len(d_ok)} / битых {len(d_bad)}, "
        f"TC ok {len(tc_ok)} / битых {len(tc_bad)}, прочее {len(other)}"
    )
    for tag, bad in (("D", d_bad), ("TC", tc_bad)):
        if bad[:3]:
            print(f"примеры битых {tag}:")
            for s in bad[:3]:
                print(f"  {s[:110]}")

    if d_ok:
        print("\nD-строки (min / avg / max):")
        vals = {n: [] for n in NAMES}
        for g in d_ok:
            for n, v in zip(NAMES, map(int, g[:9])):
                vals[n].append(v)
        for n in NAMES:
            v = vals[n]
            ref = REF.get(n)
            extra = f"  (эталон ≈{ref})" if ref is not None else ""
            print(f"  {n:>4}: {min(v)} / {statistics.mean(v):.0f} / {max(v)}{extra}")

        e = statistics.mean(vals["E"])
        deb = statistics.mean(vals["deb"])
        sync = statistics.mean(vals["sync"])
        rej = statistics.mean(vals["rej"])
        flt = statistics.mean(vals["flt"])
        gap = statistics.mean(vals["gap"])
        print("\nДиагноз:")
        if e < 100:
            print("  E ≈ 0 → НЕТ ВХОДНОГО СИГНАЛА (источник/кабель/уровень)")
        elif e > 3600:
            print("  E сильно выше нормы → лишние фронты (шум/наводки)")
        if deb > 50:
            print("  deb большой → дребезг фронтов (нужен RC/Шмитт)")
        if sync < 1 and e > 100:
            print("  sync = 0 при живом входе → синхрослово не находится (сбой декодера)")
        if rej > 1:
            print("  rej > 0 → кадры находятся, но parseFrame отвергает (сдвиг/порча)")
        if flt > 1:
            print("  flt > 0 → кадры валидны по BCD, но отклонены фильтром (чужой TC)")
        if gap > 0:
            print("  gap > 0 → срабатывал сброс фазы (разрывы сигнала)")
        if e > 100 and deb < 50 and sync >= 20 and rej < 1 and flt < 1:
            print("  вход и декодер ЗДОРОВЫ")

        last = d_ok[-1]
        raw = last[9]
        print(f"\nпоследний raw: {raw}")
        print(
            f"  b0=0x{raw[:2]} (кадры ед.), b2=0x{raw[4:6]} (сек ед.), "
            f"b4=0x{raw[8:10]} (мин ед.), b6=0x{raw[12:14]} (час ед.)"
        )

    if tc_ok:
        fps_counts: dict[str, int] = {}
        prev_idx = None
        jumps = 0
        for g in tc_ok:
            fps = int(g[5])
            h, m, s, f = map(int, g[:4])
            fps_counts[str(fps)] = fps_counts.get(str(fps), 0) + 1
            idx = ((h * 60 + m) * 60 + s) * fps + f
            if prev_idx is not None and idx != prev_idx + 1:
                jumps += 1
            prev_idx = idx
        print(f"\nTC: fps счётчик {fps_counts}, разрывов последовательности {jumps}")

        if tc_ok:
            print("первые TC:", " ".join(
                "%s:%s:%s:%s" % g[:4] for g in tc_ok[:5]))
            print("последние TC:", " ".join(
                "%s:%s:%s:%s" % g[:4] for g in tc_ok[-5:]))

    return 0


if __name__ == "__main__":
    sys.exit(main())
