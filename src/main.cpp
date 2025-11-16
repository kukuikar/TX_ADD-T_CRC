#include <Arduino.h>
#include <SoftwareSerial.h>

// --- Настройки клавиш ---
const byte NUM_KEYS = 13;

const char keyOrder[NUM_KEYS + 1] = "CBADEHGFZMOKT";
const byte BTN_PINS[NUM_KEYS] = {
  3, 6, 10, 8, 5, 2, 7, 11, 9, 4, 12, A2, A3
};

// HC-12 на аналоговых пинах как цифровые (A0=14, A1=15)
SoftwareSerial hc12(A0, A1); // RX, TX

// --- Тайминги ---
const uint32_t DEBOUNCE_MS        = 30;    // антидребезг
const uint32_t HOLD_INTERVAL      = 500;   // периодическая передача при удержании
const uint32_t IDLE_BLINK_PERIOD  = 5000;  // каждые 5 секунд
const uint32_t IDLE_BLINK_TIME    = 100;   // длительность каждой вспышки
const uint32_t IDLE_BLINK_PAUSE   = 100;   // пауза между вспышками

// --- Светодиод ---
const byte LED_PIN = 13;

// --- Состояние клавиш ---
bool     stablePressed[NUM_KEYS]    = {0};
bool     lastReading[NUM_KEYS]      = {1};
uint32_t lastDebounceTime[NUM_KEYS] = {0};

// --- Прочее ---
uint32_t lastSendTime = 0;

// Для моргания в простое (двойная вспышка)
uint32_t lastIdleBlink = 0;
byte     blinkPhase    = 0;

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

// --- Утилита: перевод nibbble в hex-символ ---
static char nibbleToHex(uint8_t v) {
  v &= 0x0F;
  if (v < 10) return '0' + v;
  return 'A' + (v - 10);
}

// --- Формируем payload "K:XXXX" ---
static void buildPayload(char* buf, uint16_t mask) {
  buf[0] = 'K';
  buf[1] = ':';
  // 4 hex-цифры, старшие сначала
  buf[2] = nibbleToHex((mask >> 12) & 0x0F);
  buf[3] = nibbleToHex((mask >> 8)  & 0x0F);
  buf[4] = nibbleToHex((mask >> 4)  & 0x0F);
  buf[5] = nibbleToHex((mask >> 0)  & 0x0F);
  buf[6] = '\0'; // на всякий случай
}

// --- Формируем пакет "K:XXXX*YY\n" ---
static void buildPacket(char* packet, const char* payload, uint8_t crc) {
  // payload всегда 6 символов: K:XXXX
  packet[0] = payload[0];
  packet[1] = payload[1];
  packet[2] = payload[2];
  packet[3] = payload[3];
  packet[4] = payload[4];
  packet[5] = payload[5];

  packet[6] = '*';
  packet[7] = nibbleToHex((crc >> 4) & 0x0F);
  packet[8] = nibbleToHex(crc & 0x0F);
  packet[9] = '\n';
  packet[10] = '\0';
}

void setup() {
  Serial.begin(9600);
  hc12.begin(9600);

  for (byte i = 0; i < NUM_KEYS; i++) {
    pinMode(BTN_PINS[i], INPUT_PULLUP);
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  uint16_t newState = 0;
  bool changed = false;
  bool anyPressed = false;

  // --- Обработка кнопок ---
  for (byte i = 0; i < NUM_KEYS; i++) {
    bool reading = (digitalRead(BTN_PINS[i]) == LOW);

    if (reading != lastReading[i]) {
      lastDebounceTime[i] = millis();
      lastReading[i] = reading;
    }

    if (millis() - lastDebounceTime[i] > DEBOUNCE_MS) {
      if (reading != stablePressed[i]) {
        stablePressed[i] = reading;
        changed = true;

        if (reading) {
          Serial.print(F("Нажата: "));
          Serial.println(keyOrder[i]);
          Serial.write(keyOrder[i]);
          Serial.write('\n');
        } else {
          Serial.print(F("Отпущена: "));
          Serial.println(keyOrder[i]);
        }
      }
    }

    if (stablePressed[i]) {
      newState |= (1U << i);  // 13 бит (0..12)
      anyPressed = true;
    }
  }

  // --- Логика LED ---
  if (anyPressed) {
    // держим кнопку — LED горит постоянно
    digitalWrite(LED_PIN, HIGH);

    // активность сбрасывает таймер простоя/мигания
    blinkPhase = 0;
    lastIdleBlink = millis();

  } else {
    // сразу гасим LED после отпускания
    // и переходим в режим простоя (двойная вспышка каждые 5 сек)
    uint32_t now = millis();
    switch (blinkPhase) {
      case 0:
        if (now - lastIdleBlink >= IDLE_BLINK_PERIOD) {
          digitalWrite(LED_PIN, HIGH);
          blinkPhase = 1;
          lastIdleBlink = now;
        }
        break;
      case 1:
        if (now - lastIdleBlink >= IDLE_BLINK_TIME) {
          digitalWrite(LED_PIN, LOW);
          blinkPhase = 2;
          lastIdleBlink = now;
        }
        break;
      case 2:
        if (now - lastIdleBlink >= IDLE_BLINK_PAUSE) {
          digitalWrite(LED_PIN, HIGH);
          blinkPhase = 3;
          lastIdleBlink = now;
        }
        break;
      case 3:
        if (now - lastIdleBlink >= IDLE_BLINK_TIME) {
          digitalWrite(LED_PIN, LOW);
          blinkPhase = 0; // цикл завершён
          lastIdleBlink = now;
        }
        break;
    }
  }

  // --- Передача ---
  bool heldEnough = (millis() - lastSendTime > HOLD_INTERVAL);

  if (newState != 0 && (changed || heldEnough)) {
    // маска по реальному числу клавиш (13 бит => 0x1FFF)
    uint16_t mask = newState & ((1U << NUM_KEYS) - 1U);

    char payload[7];   // "K:XXXX" + '\0'
    char packet[16];   // "K:XXXX*YY\n" + запас

    buildPayload(payload, mask);

    // длина payload фиксированная — 6 байт
    uint8_t crc = crc8_atm((const uint8_t*)payload, 6);

    buildPacket(packet, payload, crc);

    hc12.print(packet);
    Serial.print(packet);

    lastSendTime = millis();
  }
}
