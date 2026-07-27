#include <Arduino.h>

namespace {

constexpr uint32_t SERIAL_BAUD = 115200UL;
constexpr uint16_t SAMPLE_RATE_HZ = 50;
constexpr uint32_t SAMPLE_INTERVAL_US = 1000000UL / SAMPLE_RATE_HZ;

constexpr uint8_t ANALOG_CHANNEL_COUNT = 14;
constexpr uint8_t DIGITAL_CHANNEL_COUNT = 3;
constexpr uint8_t ADC_SAMPLES_PER_CHANNEL = 4;

// Logical A0-A11 are the clockwise infrared ring. A12 and A13 are the
// front and rear underside grayscale sensors.
const uint8_t ANALOG_PINS[ANALOG_CHANNEL_COUNT] = {
    A0, A1, A2, A3, A4, A5, A6,
    A7, A8, A9, A10, A11, A12, A13,
};

// DI0 = front-left, DI1 = front-right, DI2 = rear fence. The logical
// channel numbers intentionally do not use the Mega's physical D0/D1 pins.
const uint8_t DIGITAL_PINS[DIGITAL_CHANNEL_COUNT] = {22, 23, 24};
constexpr uint8_t DIGITAL_PIN_MODE = INPUT_PULLUP;

constexpr size_t PAYLOAD_BUFFER_SIZE = 192;

uint32_t sequence_number = 0;
uint32_t next_sample_us = 0;

uint16_t crc16CcittFalse(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}
uint16_t readAnalogAveraged(uint8_t pin) {
  // The first conversion after switching the ADC multiplexer is discarded to
  // reduce channel-to-channel carry-over.
  (void)analogRead(pin);

  uint32_t total = 0;
  for (uint8_t sample = 0; sample < ADC_SAMPLES_PER_CHANNEL; ++sample) {
    total += static_cast<uint16_t>(analogRead(pin));
  }
  return static_cast<uint16_t>(
      (total + ADC_SAMPLES_PER_CHANNEL / 2) / ADC_SAMPLES_PER_CHANNEL);
}

bool appendUnsigned(char *buffer, size_t capacity, size_t &used,
                    unsigned long value) {
  if (used >= capacity) {
    return false;
  }

  const int written =
      snprintf(buffer + used, capacity - used, ",%lu", value);
  if (written < 0 || static_cast<size_t>(written) >= capacity - used) {
    return false;
  }
  used += static_cast<size_t>(written);
  return true;
}

void sampleAndSendFrame() {
  const uint32_t sample_time_ms = millis();
  uint16_t analog_values[ANALOG_CHANNEL_COUNT];
  uint8_t digital_values[DIGITAL_CHANNEL_COUNT];

  for (uint8_t i = 0; i < ANALOG_CHANNEL_COUNT; ++i) {
    analog_values[i] = readAnalogAveraged(ANALOG_PINS[i]);
  }
  for (uint8_t i = 0; i < DIGITAL_CHANNEL_COUNT; ++i) {
    digital_values[i] = digitalRead(DIGITAL_PINS[i]) == LOW ? 0 : 1;
  }

  char payload[PAYLOAD_BUFFER_SIZE];
  int initial_written = snprintf(payload, sizeof(payload), "WF1,%lu,%lu",
                                 static_cast<unsigned long>(sequence_number),
                                 static_cast<unsigned long>(sample_time_ms));
  if (initial_written < 0 ||
      static_cast<size_t>(initial_written) >= sizeof(payload)) {
    return;
  }

  size_t used = static_cast<size_t>(initial_written);
  for (uint8_t i = 0; i < ANALOG_CHANNEL_COUNT; ++i) {
    if (!appendUnsigned(payload, sizeof(payload), used, analog_values[i])) {
      return;
    }
  }
  for (uint8_t i = 0; i < DIGITAL_CHANNEL_COUNT; ++i) {
    if (!appendUnsigned(payload, sizeof(payload), used, digital_values[i])) {
      return;
    }
  }

  const uint16_t crc = crc16CcittFalse(
      reinterpret_cast<const uint8_t *>(payload), used);
  char crc_text[5];
  snprintf(crc_text, sizeof(crc_text), "%04X", static_cast<unsigned int>(crc));

  Serial.print(payload);
  Serial.print('*');
  Serial.println(crc_text);
  ++sequence_number;
}

}  // namespace

void setup() {
  for (uint8_t i = 0; i < DIGITAL_CHANNEL_COUNT; ++i) {
    pinMode(DIGITAL_PINS[i], DIGITAL_PIN_MODE);
  }

  analogReference(DEFAULT);
  Serial.begin(SERIAL_BAUD);
  // Keep reboot recovery short for the temporary video-demonstration setup.
  // Frames are periodic, so it is harmless if the host misses the first few.
  delay(50);
  next_sample_us = micros();
}

void loop() {
  const uint32_t now_us = micros();
  if (static_cast<int32_t>(now_us - next_sample_us) < 0) {
    return;
  }

  sampleAndSendFrame();
  next_sample_us += SAMPLE_INTERVAL_US;

  // Do not emit a burst of old samples if serial output or another operation
  // ever delays the loop by more than one complete period.
  const uint32_t after_sample_us = micros();
  if (static_cast<int32_t>(after_sample_us - next_sample_us) >= 0) {
    next_sample_us = after_sample_us + SAMPLE_INTERVAL_US;
  }
}
