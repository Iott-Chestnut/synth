#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

#define MODE_BOTH
// #define MODE_SENDER
// #define MODE_RECEIVER

// IMPORTANT: octave switching disabled on purpose for debugging
// (no joystick octave, fixed octave = 4)

// ===== Timing / IDs =====
constexpr uint32_t AUDIO_FS_HZ        = 22000;
constexpr uint32_t SCAN_PERIOD_MS     = 20;
constexpr uint32_t DISPLAY_PERIOD_MS  = 100;
constexpr uint32_t CAN_ID             = 0x123;
constexpr uint8_t  FIXED_OCTAVE       = 4;

// ===== Pins =====
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

const int C0_PIN  = A2;
const int C1_PIN  = D9;
const int C2_PIN  = A6;
const int C3_PIN  = D1;
const int OUT_PIN = D11;

const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

const int KNOB_MODE = 2;
const int DEN_BIT   = 3;
const int DRST_BIT  = 4;
const int HKOW_BIT  = 5;
const int HKOE_BIT  = 6;

U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

struct {
  std::bitset<32> inputs;
  volatile uint8_t knob3Rotation; // 0..8
  uint8_t rxMsg[8];
  SemaphoreHandle_t mutex;
} sysState;

volatile uint16_t localActiveMask  = 0;  // 12-bit
volatile uint16_t remoteActiveMask = 0;  // 12-bit

static uint8_t remoteOctBuf[2][12] = {{0}};
volatile uint8_t remoteOctBufIdx = 0;

HardwareTimer sampleTimer(TIM1);

const uint32_t stepSizes[12] = {
  51076057, 54113197, 57330935, 60740010,
  64351799, 68178356, 72232452, 76527617,
  81078186, 85899346, 91007187, 96418756
};

QueueHandle_t msgInQ  = nullptr;
QueueHandle_t msgOutQ = nullptr;

void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

std::bitset<4> readCols() {
  std::bitset<4> r;
  r[0] = digitalRead(C0_PIN);
  r[1] = digitalRead(C1_PIN);
  r[2] = digitalRead(C2_PIN);
  r[3] = digitalRead(C3_PIN);
  return r;
}

void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, (rowIdx & 0x01) ? HIGH : LOW);
  digitalWrite(RA1_PIN, (rowIdx & 0x02) ? HIGH : LOW);
  digitalWrite(RA2_PIN, (rowIdx & 0x04) ? HIGH : LOW);
  digitalWrite(REN_PIN, HIGH);
}

static inline int8_t decodeKnobDelta(uint8_t prevBA, uint8_t currBA) {
  if (prevBA == currBA) return 0;
  bool prevA = (prevBA & 0x01);
  bool currA = (currBA & 0x01);
  if (prevA == currA) return 0;
  if (prevBA == 0b00 && currBA == 0b01) return +1;
  if (prevBA == 0b01 && currBA == 0b00) return -1;
  if (prevBA == 0b10 && currBA == 0b11) return -1;
  if (prevBA == 0b11 && currBA == 0b10) return +1;
  return 0;
}

static inline int32_t triFromPhase(uint32_t phase) {
  uint8_t x = (uint8_t)(phase >> 24);        
  uint8_t t = (x < 128) ? x : (uint8_t)(255 - x); 
  return ((int32_t)t << 1) - 128;              
}

void sampleISR() {
  static uint32_t phaseAccLocal[12]  = {0};
  static uint32_t phaseAccRemote[12] = {0};

  uint16_t lMask = __atomic_load_n(&localActiveMask, __ATOMIC_RELAXED);
  uint16_t rMask = __atomic_load_n(&remoteActiveMask, __ATOMIC_RELAXED);

#if defined(MODE_SENDER)
  analogWrite(OUTR_PIN, 128);
  analogWrite(OUTL_PIN, 128);
  return;
#elif defined(MODE_RECEIVER)
  lMask = 0;
#endif

  uint8_t rIdx = __atomic_load_n(&remoteOctBufIdx, __ATOMIC_RELAXED);
  const uint8_t *rOct = remoteOctBuf[rIdx];

  int32_t mix = 0;
  uint8_t voices = 0;


  for (uint8_t n = 0; n < 12; n++) {
    if (lMask & (1u << n)) {
      phaseAccLocal[n] += stepSizes[n];
      mix += triFromPhase(phaseAccLocal[n]);
      voices++;
    }
  }

  for (uint8_t n = 0; n < 12; n++) {
    if (rMask & (1u << n)) {
      (void)rOct; 
      phaseAccRemote[n] += stepSizes[n];
      mix += triFromPhase(phaseAccRemote[n]);
      voices++;
    }
  }

  if (voices == 0) {
    analogWrite(OUTR_PIN, 128);
    analogWrite(OUTL_PIN, 128);
    return;
  }

  mix /= (int32_t)voices;

  uint8_t vol = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
  if (vol > 8) vol = 8;
  mix = mix >> (8 - vol);

  int32_t out = mix + 128;
  if (out < 0) out = 0;
  if (out > 255) out = 255;

  analogWrite(OUTR_PIN, (uint8_t)out);
  analogWrite(OUTL_PIN, (uint8_t)out);
}


void CAN_RX_ISR(void) {
  uint8_t RX_Message_ISR[8] = {0};
  uint32_t ID = 0;
  CAN_RX(ID, RX_Message_ISR);
  if (msgInQ) xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void scanKeysTask(void *pvParameters) {
  const TickType_t xFrequency = SCAN_PERIOD_MS / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t prevKnob3BA = 0;
  uint16_t prevKeys12 = 0x0FFF;

  uint16_t lastMask = 0;
  uint16_t stableMask = 0;

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    std::bitset<32> localInputs;

    for (uint8_t row = 0; row < 4; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      for (uint8_t col = 0; col < 4; col++) localInputs[row * 4 + col] = cols[col];
    }


    bool A = (localInputs[3 * 4 + 0] != 0);
    bool B = (localInputs[3 * 4 + 1] != 0);
    uint8_t currKnob3BA = ((uint8_t)B << 1) | (uint8_t)A;
    int8_t delta = decodeKnobDelta(prevKnob3BA, currKnob3BA);
    prevKnob3BA = currKnob3BA;

    uint8_t newRot = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
    if (delta > 0) { if (newRot < 8) newRot++; }
    else if (delta < 0) { if (newRot > 0) newRot--; }
    __atomic_store_n(&sysState.knob3Rotation, newRot, __ATOMIC_RELAXED);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = localInputs;
    xSemaphoreGive(sysState.mutex);

    uint16_t keys12 = (uint16_t)(localInputs.to_ulong() & 0x0FFF);
    uint16_t mask = 0;
    for (uint8_t note = 0; note < 12; note++) {
      bool pressed = (((keys12 >> note) & 0x1) == 0);
      if (pressed) mask |= (1u << note);
    }

    if (mask == lastMask) stableMask = mask;
    lastMask = mask;

#if defined(MODE_RECEIVER)
    __atomic_store_n(&localActiveMask, 0u, __ATOMIC_RELAXED);
#else
    __atomic_store_n(&localActiveMask, stableMask, __ATOMIC_RELAXED);
#endif


#if !defined(MODE_RECEIVER)
    for (uint8_t note = 0; note < 12; note++) {
      bool prevPressed = (((prevKeys12 >> note) & 0x1) == 0);
      bool currPressed = (((keys12 >> note) & 0x1) == 0);
      if (prevPressed != currPressed) {
        uint8_t TX_Message[8] = {0};
        TX_Message[0] = currPressed ? (uint8_t)'P' : (uint8_t)'R';
        TX_Message[1] = FIXED_OCTAVE; // fixed
        TX_Message[2] = note;
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      }
    }
#endif

    prevKeys12 = keys12;
  }
}

void displayUpdateTask(void *pvParameters) {
  const TickType_t xFrequency = DISPLAY_PERIOD_MS / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    std::bitset<32> inputsCopy;
    uint8_t rxCopy[8];

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    inputsCopy = sysState.inputs;
    for (int i = 0; i < 8; i++) rxCopy[i] = sysState.rxMsg[i];
    xSemaphoreGive(sysState.mutex);

    uint32_t key12 = inputsCopy.to_ulong() & 0x0FFF;

    int selectedKey = -1;
    for (int k = 0; k < 12; k++) if (inputsCopy[k] == 0) selectedKey = k;

    uint8_t vol = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
    uint16_t lMask = __atomic_load_n(&localActiveMask, __ATOMIC_RELAXED);
    uint16_t rMask = __atomic_load_n(&remoteActiveMask, __ATOMIC_RELAXED);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    u8g2.drawStr(0, 10, "Keys n v oct CAN");
    u8g2.setCursor(2, 22);
    u8g2.print(key12, HEX);

    u8g2.setCursor(52, 22);
    u8g2.print(selectedKey);

    u8g2.setCursor(70, 22);
    u8g2.print(vol);

    u8g2.setCursor(90, 22);
    u8g2.print(FIXED_OCTAVE);

    u8g2.setCursor(66, 30);
    u8g2.print((char)rxCopy[0]);
    u8g2.print(rxCopy[1]);
    u8g2.print(rxCopy[2]);

    u8g2.setCursor(0, 30);
    u8g2.print("L");
    u8g2.print(lMask, HEX);
    u8g2.print(" R");
    u8g2.print(rMask, HEX);

    u8g2.sendBuffer();
    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void *pvParameters) {
  uint8_t RX_Message[8] = {0};


  for (int b = 0; b < 2; b++) {
    for (int n = 0; n < 12; n++) remoteOctBuf[b][n] = FIXED_OCTAVE;
  }
  __atomic_store_n(&remoteOctBufIdx, 0u, __ATOMIC_RELAXED);
  __atomic_store_n(&remoteActiveMask, 0u, __ATOMIC_RELAXED);

  while (1) {
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    for (int i = 0; i < 8; i++) sysState.rxMsg[i] = RX_Message[i];
    xSemaphoreGive(sysState.mutex);

#if defined(MODE_SENDER)
    continue;
#else
    uint8_t type = RX_Message[0];

    uint8_t note = RX_Message[2];

    if (note >= 12) continue;

    uint16_t rMask = __atomic_load_n(&remoteActiveMask, __ATOMIC_RELAXED);

    if (type == (uint8_t)'R') {
      rMask &= ~(1u << note);
      __atomic_store_n(&remoteActiveMask, rMask, __ATOMIC_RELAXED);
    } else if (type == (uint8_t)'P') {
      rMask |= (1u << note);
      __atomic_store_n(&remoteActiveMask, rMask, __ATOMIC_RELAXED);


    }
#endif
  }
}


void CAN_TX_Task(void *pvParameters) {
  uint8_t msgOut[8] = {0};
  while (1) {
#if defined(MODE_RECEIVER)
    vTaskDelay(10 / portTICK_PERIOD_MS);
#else
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    CAN_TX(CAN_ID, msgOut);
#endif
  }
}

static void initPinsAndDisplay() {
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);

  pinMode(C0_PIN, INPUT_PULLUP);
  pinMode(C1_PIN, INPUT_PULLUP);
  pinMode(C2_PIN, INPUT_PULLUP);
  pinMode(C3_PIN, INPUT_PULLUP);

  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);

  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);
  setOutMuxBit(KNOB_MODE, HIGH);
  setOutMuxBit(HKOE_BIT, HIGH);
  setOutMuxBit(HKOW_BIT, HIGH);
}

void setup() {
  Serial.begin(9600);

  initPinsAndDisplay();

  sysState.inputs.reset();
  sysState.knob3Rotation = 8;
  for (int i = 0; i < 8; i++) sysState.rxMsg[i] = 0;
  sysState.mutex = xSemaphoreCreateMutex();

  analogWriteResolution(8);

  msgInQ  = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(36, 8);

  CAN_Init(true);
  setCANFilter(CAN_ID, 0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_Start();

  sampleTimer.setOverflow(AUDIO_FS_HZ, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  xTaskCreate(scanKeysTask, "scanKeys", 256, NULL, 3, NULL);
  xTaskCreate(displayUpdateTask, "display", 256, NULL, 1, NULL);
  xTaskCreate(decodeTask, "decode", 256, NULL, 2, NULL);
  xTaskCreate(CAN_TX_Task, "canTx", 256, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop() {}