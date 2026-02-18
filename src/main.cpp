#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>   // <- starter CAN wrapper (per brief)

// ===== Constants =====
constexpr uint32_t AUDIO_FS_HZ = 22000;

constexpr uint32_t SCAN_PERIOD_MS    = 20;   // improved knob accuracy
constexpr uint32_t DISPLAY_PERIOD_MS = 100;  // per spec

constexpr uint32_t CAN_ID = 0x123;
constexpr uint8_t  OCTAVE = 4;              // fixed for now (per example)

// ===== Pins =====
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output mux bits
const int KNOB_MODE = 2;
const int DEN_BIT   = 3;
const int DRST_BIT  = 4;
const int HKOW_BIT  = 5;
const int HKOE_BIT  = 6;

// ===== Display =====
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// ===== Shared system state =====
struct {
  std::bitset<32> inputs;
  volatile uint8_t knob3Rotation;      // 0..8 (atomic for ISR)
  uint8_t lastRxMsg[8];                // latest received CAN payload
  SemaphoreHandle_t mutex;
} sysState;

// ===== Audio =====
volatile uint32_t currentStepSize = 0;
HardwareTimer sampleTimer(TIM1);

// 12 notes, A4=440Hz is index 9, fs=22kHz (S = (2^32*f)/fs rounded)
const uint32_t stepSizes[12] = {
  51076057, 54113197, 57330935, 60740010,
  64351799, 68178356, 72232452, 76527617,
  81078186, 85899346, 91007187, 96418756
};

// ===== OUT mux helper (starter code) =====
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

// ===== Matrix helpers =====
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

// ===== Knob decode (Knob 3: row 3 col0=A, col1=B) =====
// state encoded as {B,A} in bits: (B<<1)|A, A is bit0
static inline int8_t decodeKnobDelta(uint8_t prevBA, uint8_t currBA) {
  if (prevBA == currBA) return 0;

  bool prevA = (prevBA & 0x01);
  bool currA = (currBA & 0x01);
  if (prevA == currA) {
    // A did not toggle -> intermediate state, no count
    return 0;
  }

  // Counted transitions from the table
  if (prevBA == 0b00 && currBA == 0b01) return +1;
  if (prevBA == 0b01 && currBA == 0b00) return -1;
  if (prevBA == 0b10 && currBA == 0b11) return -1;
  if (prevBA == 0b11 && currBA == 0b10) return +1;

  // Impossible transitions ignored for now
  return 0;
}

// ===== Audio ISR =====
void sampleISR() {
  static uint32_t phaseAcc = 0;

  uint32_t step = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  phaseAcc += step;

  int32_t Vout = (int32_t)(phaseAcc >> 24) - 128; // -128..127

  // volume: Vout = Vout >> (8 - knob3Rotation)
  uint8_t vol = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
  if (vol > 8) vol = 8;
  Vout = Vout >> (8 - vol);

  uint8_t out = (uint8_t)(Vout + 128);
  analogWrite(OUTR_PIN, out);
  analogWrite(OUTL_PIN, out);
}

// ===== Tasks =====
void scanKeysTask(void *pvParameters) {
  (void)pvParameters;

  const TickType_t xFrequency = SCAN_PERIOD_MS / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // For knob decoding
  uint8_t prevKnob3BA = 0;

  // For key press/release detection (only keys 0..11)
  uint16_t prevKeys12 = 0x0FFF; // active-low, start as "all released" (1111...)

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    std::bitset<32> localInputs;

    // Scan rows 0..3 (row3 includes knob3 signals)
    for (uint8_t row = 0; row < 4; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        localInputs[row * 4 + col] = cols[col];
      }
    }

    // ----- knob3 decode -----
    bool A = (localInputs[3 * 4 + 0] != 0);
    bool B = (localInputs[3 * 4 + 1] != 0);
    uint8_t currKnob3BA = ((uint8_t)B << 1) | (uint8_t)A;

    int8_t delta = decodeKnobDelta(prevKnob3BA, currKnob3BA);
    prevKnob3BA = currKnob3BA;

    uint8_t newRot = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
    if (delta > 0) { if (newRot < 8) newRot++; }
    else if (delta < 0) { if (newRot > 0) newRot--; }
    __atomic_store_n(&sysState.knob3Rotation, newRot, __ATOMIC_RELAXED);

    // Publish inputs (mutex)
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = localInputs;
    xSemaphoreGive(sysState.mutex);

    // ----- local note selection (still plays local keys) -----
    uint32_t localStep = 0;
    for (int k = 0; k < 12; k++) {
      if (localInputs[k] == 0) localStep = stepSizes[k];
    }
    __atomic_store_n(&currentStepSize, localStep, __ATOMIC_RELAXED);

    // ----- CAN key press/release messages -----
    uint16_t keys12 = (uint16_t)(localInputs.to_ulong() & 0x0FFF); // bits are active-low

    // For each key, detect change from prev iteration
    for (uint8_t note = 0; note < 12; note++) {
      bool prevPressed = (((prevKeys12 >> note) & 0x1) == 0);
      bool currPressed = (((keys12 >> note) & 0x1) == 0);

      if (prevPressed != currPressed) {
        uint8_t TX_Message[8] = {0};

        TX_Message[0] = currPressed ? (uint8_t)'P' : (uint8_t)'R';
        TX_Message[1] = OCTAVE;
        TX_Message[2] = note;

        // Send over CAN (polling inside library)
        CAN_TX(CAN_ID, TX_Message);
      }
    }

    prevKeys12 = keys12;
  }
}

void displayUpdateTask(void *pvParameters) {
  (void)pvParameters;

  const TickType_t xFrequency = DISPLAY_PERIOD_MS / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Poll for received messages (loopback mode will receive our own TX)
    uint32_t ID = 0;
    uint8_t RX_Message[8] = {0};

    while (CAN_CheckRXLevel()) {
      CAN_RX(ID, RX_Message);

      // store latest received message in sysState for display
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      for (int i = 0; i < 8; i++) sysState.lastRxMsg[i] = RX_Message[i];
      xSemaphoreGive(sysState.mutex);
    }

    // Copy shared state for display
    std::bitset<32> inputsCopy;
    uint8_t msgCopy[8];
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    inputsCopy = sysState.inputs;
    for (int i = 0; i < 8; i++) msgCopy[i] = sysState.lastRxMsg[i];
    xSemaphoreGive(sysState.mutex);

    uint32_t key12 = inputsCopy.to_ulong() & 0x0FFF;

    int selectedKey = -1;
    for (int k = 0; k < 12; k++) {
      if (inputsCopy[k] == 0) selectedKey = k;
    }

    uint8_t vol = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);

    // Display
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    u8g2.drawStr(0, 10, "Keys note vol  CAN");
    u8g2.setCursor(2, 22);
    u8g2.print(key12, HEX);

    u8g2.setCursor(52, 22);
    u8g2.print(selectedKey);

    u8g2.setCursor(80, 22);
    u8g2.print(vol);

    // Show latest CAN message (per brief format)
    u8g2.setCursor(66, 30);
    u8g2.print((char)msgCopy[0]);
    u8g2.print(msgCopy[1]);
    u8g2.print(msgCopy[2]);

    u8g2.sendBuffer();

    // Spec: LED toggle in display task
    digitalToggle(LED_BUILTIN);
  }
}

// ===== Setup / Loop =====
void setup() {
  // GPIO
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

  // Display init (starter)
  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);
  setOutMuxBit(KNOB_MODE, HIGH);

  // optional routing enables (harmless if unused)
  setOutMuxBit(HKOE_BIT, HIGH);
  setOutMuxBit(HKOW_BIT, HIGH);

  Serial.begin(9600);
  Serial.println("Part 2 CAN polling start");

  // Init sysState + mutex (before scheduler)
  sysState.inputs.reset();
  sysState.knob3Rotation = 8; // loud by default
  for (int i = 0; i < 8; i++) sysState.lastRxMsg[i] = 0;
  sysState.mutex = xSemaphoreCreateMutex();

  // Audio
  analogWriteResolution(8);
  sampleTimer.setOverflow(AUDIO_FS_HZ, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  // CAN init (loopback = true for testing)
  CAN_Init(true);
  setCANFilter(CAN_ID, 0x7ff);
  CAN_Start();

  // Tasks
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(scanKeysTask, "scanKeys", 256, NULL, 2, &scanKeysHandle);

  TaskHandle_t displayHandle = NULL;
  xTaskCreate(displayUpdateTask, "display", 256, NULL, 1, &displayHandle);

  // Scheduler (must be last)
  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS: unused
}
