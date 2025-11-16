// ===== RX for Arduino UNO (HC-12 + 13 relays, flexible mapping) =====
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ctype.h>   // isxdigit

// ---------- HC-12 ----------
#define HC12_RX_PIN A0
#define HC12_TX_PIN A1
SoftwareSerial hc12(HC12_RX_PIN, HC12_TX_PIN);
#define HC12_BAUD 9600

// ---------- Конфигурация реле ----------
struct RelayConfig {
  char name;
  byte pin;
};

const RelayConfig relays[] = {
  {'C', 2},
  {'B', 3},
  {'A', 4},
  {'D', 5},
  {'E', 6},
  {'H', 7},
  {'G', 8},
  {'F', 9},
  {'Z', 10},
  {'M', 11},
  {'O', 12},
  {'K', A2},
  {'T', A3}
};

const byte RELAY_COUNT = sizeof(relays) / sizeof(relays[0]);

// LOW-активные модули
const byte RELAY_ACTIVE_LEVEL   = LOW;
const byte RELAY_INACTIVE_LEVEL = HIGH;

// ---------- Индикация ----------
const byte LED_PIN = 13;
const bool LED_ACTIVE_HIGH = true;

// ---------- Таймаут ----------
#define RELAY_TIMEOUT_MS 500

// ---------- Внутренние ----------
char rxBuf[32];       // небольшой запас
byte rxIdx = 0;
uint32_t lastCmdMs = 0;
uint16_t lastMask = 0;

// ---------- CRC-8 (ATM/HEC) ----------
static uint8_t crc8_atm(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;             // init=0x00
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07); // poly=0x07
      else            crc <<= 1;
    }
  }
  return crc;                      // no reflect, no xorout
}

// печать двухзначного HEX с ведущим нулём
static void printHex2(uint8_t v) {
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

// ---------- Служебные функции ----------
inline void setLed(bool on) {
  digitalWrite(LED_PIN, LED_ACTIVE_HIGH ? (on ? HIGH : LOW)
                                        : (on ? LOW  : HIGH));
}

inline void writeRelay(byte pin, bool on) {
  digitalWrite(pin, on ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
}

void applyRelayMask(uint16_t mask) {
  for (byte i = 0; i < RELAY_COUNT; i++) {
    bool on = (mask >> i) & 0x1;
    writeRelay(relays[i].pin, on);
  }
}

void allRelaysOff() {
  applyRelayMask(0);
}

// ---------- Парсер команд с проверкой CRC ----------
bool tryParseAndApply(const char* line) {
  // trim ведущие пробелы
  while (*line == ' ' || *line == '\t') line++;

  // ожидаем "K:"
  if (line[0] != 'K' || line[1] != ':') return false;

  // ищем звёздочку-разделитель CRC
  const char* star = strchr(line, '*');
  if (!star) {
    Serial.println(F("RX: no CRC separator '*'"));
    return false;
  }

  // payload = все байты ДО '*', CRC считаем по ASCII payload
  const uint8_t* payload = reinterpret_cast<const uint8_t*>(line);
  size_t payload_len = (size_t)(star - line);

  // читаем 2 hex-цифры CRC после '*'
  if (!isxdigit((unsigned char)star[1]) || !isxdigit((unsigned char)star[2])) {
    Serial.println(F("RX: CRC hex parse error"));
    return false;
  }
  char crcHex[3] = { star[1], star[2], 0 };
  uint8_t rx_crc = (uint8_t)strtoul(crcHex, nullptr, 16);

  // считаем CRC по payload
  uint8_t calc_crc = crc8_atm(payload, payload_len);

  if (calc_crc != rx_crc) {
    Serial.print(F("RX: CRC mismatch calc="));
    printHex2(calc_crc);
    Serial.print(F(" recv="));
    printHex2(rx_crc);
    Serial.println();
    return false;
  }

  // успешная проверка CRC
  Serial.print(F("RX: CRC OK calc="));
  printHex2(calc_crc);
  Serial.print(F(" recv="));
  printHex2(rx_crc);
  Serial.println();

  // извлекаем маску из подстроки между "K:" и '*'
  const char* maskStart = line + 2;         // после "K:"
  size_t maskLen = (size_t)(star - maskStart);
  if (maskLen == 0 || maskLen > 4) {        // передаётся %03X, но позволим до 4
    Serial.println(F("RX: mask length error"));
    return false;
  }
  char maskHex[5] = {0,0,0,0,0};
  memcpy(maskHex, maskStart, maskLen);
  uint16_t mask = (uint16_t)(strtoul(maskHex, nullptr, 16) & 0x1FFF);

  // применяем
  applyRelayMask(mask);
  lastMask  = mask;
  lastCmdMs = millis();
  setLed(mask != 0);

  // --- отладка ---
  Serial.print(F("Mask: 0x"));
  Serial.print(mask, HEX);
  Serial.print(F("  Active: "));
  bool any = false;
  for (byte i = 0; i < RELAY_COUNT; i++) {
    if ((mask >> i) & 1) {
      any = true;
      Serial.print(relays[i].name);
      Serial.print(' ');
    }
  }
  if (!any) Serial.print("none");
  Serial.println();

  return true;
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  hc12.begin(HC12_BAUD);

  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  // Инициализация реле
  for (byte i = 0; i < RELAY_COUNT; i++) {
    pinMode(relays[i].pin, OUTPUT);
    writeRelay(relays[i].pin, false);
    Serial.print(relays[i].name);
    Serial.print(F(" → pin "));
    Serial.println(relays[i].pin);
  }

  Serial.println(F("RX UNO ready (HC-12 + 13 relays, CRC-checked)"));
}

// ---------- loop ----------
void loop() {
  // Приём построчно
  while (hc12.available()) {
    char c = (char)hc12.read();
    if (c == '\n') {
      rxBuf[rxIdx] = '\0';
      tryParseAndApply(rxBuf);
      rxIdx = 0;
    } else if (c != '\r') {
      if (rxIdx < (sizeof(rxBuf) - 1)) rxBuf[rxIdx++] = c;
      else rxIdx = 0; // сброс при переполнении
    }
  }

  // Таймаут — отключаем все реле
  uint32_t now = millis();
  if (now - lastCmdMs > RELAY_TIMEOUT_MS) {
    if (lastMask != 0) {
      allRelaysOff();
      setLed(false);
      lastMask = 0;
    }
    lastCmdMs = now;
  }
}
