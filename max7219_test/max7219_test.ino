/*
 * Тест сегментного индикатора на MAX7219 (8 разрядов) + Arduino Nano.
 *
 * Каждая ячейка (разряд) показывает свой номер: 01234567.
 * Затем раз в секунду цифры сдвигаются по кругу — так видно,
 * что работает каждый разряд и нет «битых» сегментов.
 *
 * Подключение (аппаратный SPI):
 *   MAX7219 VCC -> 5V
 *   MAX7219 GND -> GND
 *   MAX7219 DIN -> D11 (MOSI)
 *   MAX7219 CS  -> D10 (SS)
 *   MAX7219 CLK -> D13 (SCK)
 *
 * Библиотека не требуется — регистры MAX7219 пишутся напрямую.
 */

#include <SPI.h>

const uint8_t PIN_CS = 10;   // CS (LOAD)

// Регистры MAX7219
const uint8_t REG_DECODE   = 0x09; // режим декодирования BCD
const uint8_t REG_INTENSITY= 0x0A; // яркость 0..15
const uint8_t REG_SCANLIMIT= 0x0B; // число разрядов - 1
const uint8_t REG_SHUTDOWN = 0x0C; // 0 = сон, 1 = работа
const uint8_t REG_TEST     = 0x0F; // тест всех сегментов

// Коды цифр 0..9 для decode-mode (BCD)
const uint8_t NUM_DIGITS = 8;

void max7219Write(uint8_t reg, uint8_t value) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(reg);
  SPI.transfer(value);
  digitalWrite(PIN_CS, HIGH);
}

void max7219Init() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin();

  max7219Write(REG_TEST,      0x00); // выключить тест
  max7219Write(REG_DECODE,    0xFF); // BCD-декодирование на всех разрядах
  max7219Write(REG_SCANLIMIT, 0x07); // все 8 разрядов
  max7219Write(REG_INTENSITY, 0x04); // средняя яркость
  max7219Write(REG_SHUTDOWN,  0x01); // нормальный режим
  clearDisplay();
}

void clearDisplay() {
  for (uint8_t d = 0; d < NUM_DIGITS; d++) {
    max7219Write(d + 1, 0x0F); // 0x0F = пустой разряд
  }
}

// Вывести цифру num (0..9) в разряд pos (0..7), pos 0 — крайний правый
void showDigit(uint8_t pos, uint8_t num) {
  max7219Write(pos + 1, num);
}

void setup() {
  max7219Init();

  // Полный тест: все сегменты горят 1 секунду
  max7219Write(REG_TEST, 0x01);
  delay(1000);
  max7219Write(REG_TEST, 0x00);

  // Стартовое состояние: каждая ячейка показывает свой номер
  for (uint8_t pos = 0; pos < NUM_DIGITS; pos++) {
    showDigit(pos, pos);
  }
}

void loop() {
  delay(1000);

  // Сдвиг всех цифр на один разряд вправо по кругу.
  // Если каждая ячейка в итоге побывала в каждой позиции и
  // цифры читаются чётко — дисплей исправен.
  static uint8_t buf[NUM_DIGITS];
  for (uint8_t pos = 0; pos < NUM_DIGITS; pos++) buf[pos] = pos;

  static uint8_t shift = 0;
  shift = (shift + 1) % NUM_DIGITS;

  for (uint8_t pos = 0; pos < NUM_DIGITS; pos++) {
    uint8_t src = (pos + shift) % NUM_DIGITS;
    showDigit(pos, buf[src]);
  }
}
