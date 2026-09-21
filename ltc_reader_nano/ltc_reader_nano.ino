// ============================================================================
// LTC Reader — Arduino Nano + MAX7219
// Декодер SMPTE LTC: аналоговый компаратор (D7/D6) + Timer1 Input Capture,
// вывод HH.MM.SS.FF на 8-значный 7-сегментный MAX7219 (SPI: D11/D13, CS D10).
//
// Структура: блоки 1–9 по PLAN.md §6, все реализованы.
// Раскладка битов кадра (LSB-first) и логика декодирования проверены
// на тестовом WAV — tools/ltc_wav_check.py.
// ============================================================================

#include <SPI.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// ============================================================================
// [Блок 1] Константы
// ============================================================================

#define DEBUG 0                     // 1 = Serial-лог кадров, 115200

const uint8_t  PIN_CS     = 10;     // MAX7219 LOAD/CS (MOSI=11, SCK=13 — HW SPI)

// sync word (биты 64–79, на проводе LSB-first): 0011111111111101
// в раскладке регистра «первый бит кадра = LSB байта 0»: byte8, byte9
const uint8_t SYNC_B8 = 0xFC;
const uint8_t SYNC_B9 = 0xBF;
const uint16_t MIN_EDGE   = 160;    // антидребезг: 80 мкс / тик 0.5 мкс
const uint8_t  BUF_SIZE   = 64;     // кольцевой буфер фронтов (степень двойки!)
const uint8_t  FPS_HYST   = 4;      // смена fps: N подряд классификаций в пользу нового
const uint16_t GAP_RESET_MS = 5;    // нет фронтов дольше → разрыв: сброс фазы декодера

// Регистры MAX7219
const uint8_t MAX_DECODE    = 0x09;
const uint8_t MAX_INTENSITY = 0x0A;
const uint8_t MAX_SCANLIMIT = 0x0B;
const uint8_t MAX_SHUTDOWN  = 0x0C;
const uint8_t MAX_TEST      = 0x0F;

// ============================================================================
// [Блок 2] Глобальное состояние
// ============================================================================

// --- Кольцевой буфер меток фронтов: ISR (писатель) → loop (читатель) ---
volatile uint16_t edgeBuf[BUF_SIZE];
volatile uint8_t  edgeHead = 0;      // индекс записи (только ISR)
volatile uint8_t  edgeTail = 0;      // индекс чтения (только loop)

// --- Декодер bi-phase mark ---
uint16_t lastEdge    = 0;            // метка предыдущего фронта (тики Timer1)
bool     haveEdge    = false;        // есть предыдущий фронт
bool     halfPending = false;        // пойман первый короткий интервал пары («1»)
uint32_t lastEdgeMs  = 0;            // millis() последнего фронта (детект разрыва)

// --- 80-битный сдвиговый аккумулятор кадра ---
uint8_t  frameBits[10];              // живой сдвиговый регистр (последние 80 бит)
uint8_t  frameOut[10];               // защёлкнутый полный кадр для парсера
bool     frameReady  = false;        // sync word замкнул кадр → loop заберёт

// --- Результат ---
struct TimeCode {
  uint8_t h, m, s, f;                // HH MM SS FF
  bool    drop;                      // флаг drop frame (бит 6)
};
TimeCode tc;                         // последний декодированный TC
bool     tcValid     = false;        // хоть один кадр декодирован

// --- Авто-fps / потеря сигнала ---
uint8_t  fps         = 0;            // 24/25/30; 0 = не определён
uint32_t lastFrameMs = 0;            // millis() последнего декодированного кадра

// --- Диагностика входа (только DEBUG): счётчики за 1 с ---
uint16_t dbgEdges = 0;               // фронтов вычитано из буфера
uint16_t dbgDeb   = 0;               // отброшено антидребезгом
uint16_t dbgShort = 0;               // классифицировано коротких интервалов
uint16_t dbgLong  = 0;               // классифицировано длинных интервалов
uint16_t dbgSync  = 0;               // совпадений sync word
uint16_t dbgRej   = 0;               // parseFrame отверг кадр
uint16_t dbgFlt   = 0;               // фильтр правдоподобия отклонил кадр
uint16_t dbgGap   = 0;               // сбросов фазы (разрыв сигнала)
uint32_t dbgMs    = 0;               // millis() прошлой диагностики

// ============================================================================
// [Блок 8] MAX7219 — прямой SPI
// ============================================================================

void max7219_send(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); // 4 МГц
  digitalWrite(PIN_CS, LOW);
  SPI.transfer16((uint16_t)(reg << 8) | val);     // адрес + данные
  digitalWrite(PIN_CS, HIGH);                     // LOAD: защёлкивание по фронту
  SPI.endTransaction();
}

void renderTC() {
  // Code B: 0..9 = цифра, бит 7 = десятичная точка → HH.MM.SS.FF.
  // Модуль стенда зеркалит разряды (reg1 = крайний правый), поэтому d[0]
  // уходит в reg8: слева направо выходит d0 d1. d2 d3. d4 d5. d6 d7.
  const uint8_t d[8] = {
    (uint8_t)(tc.h / 10), (uint8_t)(tc.h % 10),
    (uint8_t)(tc.m / 10), (uint8_t)(tc.m % 10),
    (uint8_t)(tc.s / 10), (uint8_t)(tc.s % 10),
    (uint8_t)(tc.f / 10), (uint8_t)(tc.f % 10)
  };
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t v = d[i] & 0x0F;
    if (i == 1 || i == 3 || i == 5) v |= 0x80;    // точки после HH, MM, SS
    max7219_send((uint8_t)(8 - i), v);            // зеркальный порядок разрядов
  }
}

// ============================================================================
// [Блок 3] setup() — инициализация железа
// ============================================================================

void setup() {
#if DEBUG
  pinMode(0, INPUT_PULLUP);   // до Serial.begin: D1(TX) в сбросе/паузах плавает
  pinMode(1, INPUT_PULLUP);   // и питает RXD адаптера мусором/break → флуд NUL
  Serial.begin(115200);
#endif

  // --- SPI + MAX7219 (настройки SPI — внутри max7219_send, блок 8) ---
  SPI.begin();
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  max7219_send(MAX_SHUTDOWN, 0x00);   // shutdown на время настройки
  max7219_send(MAX_SCANLIMIT, 0x07);  // все 8 цифр
  max7219_send(MAX_DECODE, 0xFF);     // Code B на все цифры
  max7219_send(MAX_INTENSITY, 0x03);  // яркость 0..15: ниже = меньше импульсных
                                      // токов = меньше наводок на вход (стенд)
  max7219_send(MAX_TEST, 0x00);       // display test выкл
  for (uint8_t d = 1; d <= 8; d++)
    max7219_send(d, 0x0A);            // Code B 0x0A = '-' → "--------"
  max7219_send(MAX_SHUTDOWN, 0x01);   // включить индикатор

  // --- Захват LTC: Timer1 Input Capture ← аналоговый компаратор ---
  TCCR1A = 0x00;                      // normal mode
  TCCR1B = _BV(ICNC1)                 // noise canceller (4 сэмпла)
         | _BV(ICES1)                 // старт: восходящий фронт
         | _BV(CS11);                 // prescaler 8 → тик 0.5 мкс
  TIFR1  = _BV(ICF1);                 // сброс зависшего флага
  TIMSK1 = _BV(ICIE1);                // прерывание захвата — пуск

  ACSR   = _BV(ACIC);                 // выход компаратора → Input Capture
                                      // (ACIE=0: прерывание компаратора не нужно)
  DIDR1  = _BV(AIN0D) | _BV(AIN1D);   // цифровые буферы AIN0/AIN1 прочь

#if DEBUG
  Serial.println(F("LTC reader init OK"));
#endif
}

// ============================================================================
// [Блок 4] ISR захвата фронтов
// ============================================================================

ISR(TIMER1_CAPT_vect) {
  uint8_t h    = edgeHead;
  uint8_t next = (uint8_t)((h + 1) & (BUF_SIZE - 1));
  if (next != edgeTail) {             // буфер не полон — пишем метку
    edgeBuf[h] = ICR1;
    edgeHead = next;
  }                                   // полон: фронт теряем (loop не успевает)
  TCCR1B ^= _BV(ICES1);               // следующий захват — противоположный фронт
}

// ============================================================================
// [Блок 5] Декодер bi-phase mark
// ============================================================================

// Адаптивный порог короткий/длинный (тики Timer1, 1 тик = 0.5 мкс).
// Старт 25 fps: полбита 500 тиков, бит 1000; EMA 1/4 адаптирует к скорости.
static uint16_t shortAvg = 500;
static uint16_t longAvg  = 1000;
static uint16_t thrShort = 750;      // (shortAvg + longAvg) / 2 ≈ 0.75 периода

// Сдвиг бита в 80-битный регистр. Раскладка проверена на тестовом WAV
// (LSB-first): бит 0 кадра = LSB frameBits[0]; sync = байты 8–9.
static void shiftBit(uint8_t bit) {
  uint8_t carry = (uint8_t)(bit ? 0x80 : 0x00); // новый бит → MSB (поз. 79)
  for (int8_t j = 9; j >= 0; j--) {             // от старшего байта к младшему
    uint8_t out = (uint8_t)(frameBits[j] & 0x01); // LSB байта j → MSB байта j-1
    frameBits[j] = (uint8_t)((frameBits[j] >> 1) | carry);
    carry = (uint8_t)(out ? 0x80 : 0x00);
  }
  if (frameBits[8] == SYNC_B8 && frameBits[9] == SYNC_B9) {
    for (uint8_t i = 0; i < 10; i++) frameOut[i] = frameBits[i];
    frameReady = true;                          // кадр полный — loop заберёт
#if DEBUG
    dbgSync++;
#endif
  }
}

void processEdges() {
  bool saw = false;
  while (edgeTail != edgeHead) {
    uint16_t now = edgeBuf[edgeTail];
    edgeTail = (uint8_t)((edgeTail + 1) & (BUF_SIZE - 1));
    saw = true;
#if DEBUG
    dbgEdges++;
#endif

    if (!haveEdge) {                 // первый фронт — только база отсчёта
      lastEdge = now;
      haveEdge = true;
      continue;
    }
    uint16_t delta = (uint16_t)(now - lastEdge); // переполнение Timer1 не мешает
    lastEdge = now;
    if (delta < MIN_EDGE) {                      // дребезг — игнор
#if DEBUG
      dbgDeb++;
#endif
      continue;
    }

    if (delta < thrShort) {          // короткий = полбита
#if DEBUG
      dbgShort++;
#endif
      if (halfPending) {             // пара коротких → бит «1»
        halfPending = false;
        shortAvg = (uint16_t)((shortAvg * 3 + delta) >> 2);
        thrShort = (uint16_t)((shortAvg + longAvg) >> 1);
        shiftBit(1);
      } else {
        halfPending = true;          // ждём второй короткий
      }
    } else {                         // длинный = целый бит → «0»
      // короткий+длинный = потеря фронта: бит-граница сомнительна,
      // синхронизацию восстановит sync-детектор на текущем/следующем кадре
      halfPending = false;
#if DEBUG
      dbgLong++;
#endif
      longAvg = (uint16_t)((longAvg * 3 + delta) >> 2);
      thrShort = (uint16_t)((shortAvg + longAvg) >> 1);
      shiftBit(0);
    }
  }
  if (saw) lastEdgeMs = millis();    // фронты есть — сигнал жив
}

// ============================================================================
// [Блок 6] Парсер BCD-кадра
// ============================================================================

bool parseFrame() {
  // Стандартный SMPTE 12M LTC (LSB-first, проверено по libltc): user bits
  // вклиниваются между нибблами, поэтому tens каждого поля BCD лежит в
  // СЛЕДУЮЩЕМ байте, а не в старшем ниббле своего:
  //   b0=кадры ед., b1[1:0]=кадры дес., b1[2]=DF, b1[3]=CF,
  //   b2=секунды ед., b3[2:0]=секунды дес.,
  //   b4=минуты ед.,  b5[2:0]=минуты дес.,
  //   b6=часы ед.,    b7[1:0]=часы дес., b8..b9=sync.
  // ВНИМАНИЕ: старшие нибблы b1/b3/b5/b7 (и весь b1..) — user bits, НЕ время!
  uint8_t fU =  frameOut[0] & 0x0F;        // биты 0–3:   кадры, единицы
  uint8_t fT =  frameOut[1] & 0x03;        // биты 8–9:   кадры, десятки
  bool    df = (frameOut[1] >> 2) & 1;     // бит 10:     drop frame
  uint8_t sU =  frameOut[2] & 0x0F;        // биты 16–19: секунды, единицы
  uint8_t sT =  frameOut[3] & 0x07;        // биты 24–26: секунды, десятки
  uint8_t mU =  frameOut[4] & 0x0F;        // биты 32–35: минуты, единицы
  uint8_t mT =  frameOut[5] & 0x07;        // биты 40–42: минуты, десятки
  uint8_t hU =  frameOut[6] & 0x0F;        // биты 48–51: часы, единицы
  uint8_t hT =  frameOut[7] & 0x03;        // биты 56–57: часы, десятки

  if (fU > 9 || fT > 2 || sU > 9 || sT > 5 ||    // не BCD — мусорный кадр
      mU > 9 || mT > 5 || hU > 9 || hT > 2)
    return false;

  uint8_t f = fT * 10 + fU;
  uint8_t s = sT * 10 + sU;
  uint8_t m = mT * 10 + mU;
  uint8_t h = hT * 10 + hU;
  if (h > 23 || m > 59 || s > 59 || f >= (fps ? fps : 30))
    return false;

  tc.h = h; tc.m = m; tc.s = s; tc.f = f; tc.drop = df;
  return true;
}

// ============================================================================
// [Блок 6b] Фильтр правдоподобия таймкода
// ============================================================================

// Пара «лишний/потерянный фронт» на входе даёт одиночную битовую ошибку:
// кадр остаётся валидным по BCD, но время в нём чужое (напр. час 02 → 04).
// Такой кадр не показываем. Одиночный сбой не должен попадать на дисплей,
// но настоящие смены источника (перезапуск файла, другой поток) проходить
// обязаны — поэтому после потери сигнала (>500 мс) или смены fps фильтр
// перезахватывается, а устойчиво «другой» ряд кадров принимается.
static bool tcPlausible() {
  static uint32_t prevIdx   = 0;       // номер последнего принятого кадра
  static uint8_t  prevFps   = 0;       // fps, при котором считался prevIdx
  static bool     havePrev  = false;
  static uint8_t  rejectRun = 0;       // подряд отброшено

  bool reacquire = (!fps || !havePrev || fps != prevFps ||
                    (millis() - lastFrameMs) > 500);
  if (reacquire) rejectRun = 0;

  uint32_t idx = (((uint32_t)tc.h * 60 + tc.m) * 60 + tc.s) * (fps ? fps : 30) + tc.f;

  bool ok;
  if (reacquire) {
    ok = true;                                  // (ре)захват — принимаем любой
  } else {
    int32_t d = (int32_t)(idx - prevIdx);
    ok = (d >= 0 && d <= 4);                    // вперёд на 1..2 кадра, терпим пропуски
    if (!ok && ++rejectRun >= 8) ok = true;     // ряд «не туда» — смена источника
  }
  if (ok) {
    prevIdx  = idx;
    prevFps  = fps;
    havePrev = true;
    rejectRun = 0;
  }
  return ok;
}

// ============================================================================
// [Блок 7] Автоопределение fps
// ============================================================================

void updateFps() {
  // Период кадра = интервал между готовыми кадрами (sync words), micros(),
  // EMA 1/4, классификация после 8 кадров. 29.97 и 30 по периоду не
  // различить (33.367 vs 33.333 мс) — оба дают fps=30, DF виден в tc.drop.
  // Вызывается на КАЖДОМ готовом кадре (см. loop), независимо от валидации
  // parseFrame: иначе отбраковка кадров растягивает интервал и уводит fps.
  static uint32_t lastUs = 0;
  static bool     have   = false;
  static uint32_t avgUs  = 0;
  static uint8_t  n      = 0;
  static uint8_t  cand   = 0;          // кандидат на смену fps
  static uint8_t  candN  = 0;          // сколько классификаций подряд за него

  uint32_t now = micros();
  if (have) {
    uint32_t d = now - lastUs;
    bool ok = (d >= 15000UL && d <= 70000UL);   // правдоподобно (varispeed ~0.5–2×)
    // Потеря sync word даёт двойной интервал — отсеиваем его, пока fps уже
    // захвачен: иначе EMA растягивается и на время уводит скорость вниз, что
    // ошибочно забраковало бы кадры 24–29 у 30fps-источника.
    if (ok && n >= 8 && d > (uint32_t)((avgUs * 3UL) >> 1))
      ok = false;
    if (ok) {
      avgUs = n ? (avgUs * 3 + d) >> 2   // выброс — fps не трогаем
                : d;
      if (n < 8) n++;
      if (n >= 8) {
        uint8_t f = avgUs > 40800UL ? 24   // ~41.7 мс
                  : avgUs > 36000UL ? 25   // ~40.0 мс
                  : 30;                    // ~33.3 мс
        if (f == fps || !fps) {            // подтверждение / первичный захват
          fps = f;
          candN = 0;
        } else if (f == cand) {
          if (++candN >= FPS_HYST) {       // скорость устойчиво другая — меняем
            fps = f;
            candN = 0;
          }
        } else {
          cand = f;                        // новый кандидат — считаем сначала
          candN = 1;
        }
      }
    }
  }
  lastUs = now;
  have = true;
}

// ============================================================================
// [Блок 7b] Мягкий ре-инит декодера
// ============================================================================

// Аналог Reset, но без сброса железа: возвращает адаптивные пороги к старту
// и обнуляет фазу/регистр. Нужен, когда сигнал есть, а валидных кадров нет —
// пауза/переходный процесс могут испортить shortAvg/longAvg/thrShort, и без
// этого декодер «залипает» навсегда (лечится только перезагрузкой).
static void reinitDecoder() {
  shortAvg = 500;
  longAvg  = 1000;
  thrShort = 750;
  haveEdge    = false;
  halfPending = false;
  frameReady  = false;
  for (uint8_t i = 0; i < 10; i++) frameBits[i] = 0;
}

// ============================================================================
// [Блок 9] loop() — главный цикл
// ============================================================================

void loop() {
#if DEBUG
  // --- 0. диагностика раз в секунду: вход, классификация, sync, сырой кадр ---
  if (millis() - dbgMs >= 1000) {
    dbgMs = millis();
    Serial.print(F("D E="));   Serial.print(dbgEdges);
    Serial.print(F(" deb="));  Serial.print(dbgDeb);
    Serial.print(F(" S="));    Serial.print(dbgShort);
    Serial.print(F(" L="));    Serial.print(dbgLong);
    Serial.print(F(" sync=")); Serial.print(dbgSync);
    Serial.print(F(" rej="));  Serial.print(dbgRej);
    Serial.print(F(" flt="));  Serial.print(dbgFlt);
    Serial.print(F(" gap="));  Serial.print(dbgGap);
    Serial.print(F(" thr="));  Serial.print(thrShort);
    Serial.print(F(" raw="));
    for (uint8_t i = 0; i < 10; i++) {
      if (frameOut[i] < 0x10) Serial.print('0');
      Serial.print(frameOut[i], HEX);
    }
    Serial.println();
    dbgEdges = dbgDeb = dbgShort = dbgLong = dbgSync = 0;
    dbgRej = dbgFlt = dbgGap = 0;
  }
#endif

  // --- 1. декодирование ---
  processEdges();

  // --- 1b. разрыв сигнала: фронтов нет дольше GAP_RESET_MS → сброс фазы.
  // Иначе первый фронт после паузы (огромный интервал) даёт фантомный бит и
  // сдвигает весь поток на 1 бит: sync находится, но кадры «съезжают», и
  // parseFrame бракует всё — дисплей навсегда замирает на старом TC.
  if (haveEdge && (millis() - lastEdgeMs) > GAP_RESET_MS) {
    haveEdge    = false;             // следующий фронт — новая база отсчёта
    halfPending = false;
#if DEBUG
    dbgGap++;
#endif
  }

  // --- 1c. сторож: сигнал идёт, но валидных кадров нет > 400 мс → мягкий
  // ре-инит (пороги/фаза), чтобы выйти из залипшего состояния без Reset.
  static uint32_t lastReinitMs = 0;
  if ((millis() - lastEdgeMs) < 100 &&          // фронты идут → сигнал есть
      (millis() - lastFrameMs) > 400 &&         // но валидных кадров нет
      (millis() - lastReinitMs) > 400) {
    lastReinitMs = millis();
    reinitDecoder();
#if DEBUG
    Serial.println(F("REINIT"));
#endif
  }

  if (frameReady) {
    frameReady = false;
    updateFps();                      // период считаем на каждом sync word
    bool parsed = parseFrame();
    if (parsed && tcPlausible()) {
      renderTC();
      lastFrameMs = millis();
      tcValid = true;
#if DEBUG
      Serial.print(F("TC "));
      if (tc.h < 10) Serial.print('0');
      Serial.print(tc.h); Serial.print(':');
      if (tc.m < 10) Serial.print('0');
      Serial.print(tc.m); Serial.print(':');
      if (tc.s < 10) Serial.print('0');
      Serial.print(tc.s); Serial.print(':');
      if (tc.f < 10) Serial.print('0');
      Serial.print(tc.f);
      if (tc.drop) Serial.print(F(" DF"));
      Serial.print(F(" fps="));
      Serial.println(fps);
#endif
    }
#if DEBUG
    else if (!parsed) dbgRej++;       // кадр не прошёл BCD/диапазоны
    else              dbgFlt++;       // отклонён фильтром правдоподобия
#endif
  }

  // --- 2. потеря сигнала: > 500 мс — мигание последним TC ---
  static uint32_t lastBlinkMs = 0;
  static bool     blinkOn     = true;
  if (tcValid && (millis() - lastFrameMs) > 500) {
    if ((millis() - lastBlinkMs) >= 250) {
      lastBlinkMs = millis();
      blinkOn = !blinkOn;
      max7219_send(MAX_SHUTDOWN, blinkOn ? 0x01 : 0x00);
    }
  } else if (!blinkOn) {             // сигнал вернулся — включить дисплей
    blinkOn = true;
    max7219_send(MAX_SHUTDOWN, 0x01);
  }
}
