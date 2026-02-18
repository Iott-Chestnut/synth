#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>

// ===== Constants =====
constexpr uint32_t AUDIO_FS_HZ = 22000;

// Task periods (per Part 2 knob suggestion)
constexpr uint32_t SCAN_PERIOD_MS = 20;     // was 50, now 20ms to improve knob decode
constexpr uint32_t DISPLAY_PERIOD_MS = 100; // per spec (also toggles LED)

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

// ===== Shared system state (per Part 2 Section 1) =====
struct {
  std::bitset<32> inputs;
  volatile uint8_t knob3Rotation;     // 0..8 (read by ISR via atomic)
  SemaphoreHandle_t mutex;            // FreeRTOS mutex handle
} sysState;

// ===== Audio =====
volatile uint32_t currentStepSize = 0;
HardwareTimer sampleTimer(TIM1);

// 12 notes (C..B), equal temperament, A4=440Hz is index 9, fs=22kHz
// S = (2^32 * f)/fs rounded
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

// ===== Knob decode helper (Knob 3 = row 3, col0=A, col1=B) =====
// We represent state as {B,A} in bits: (B<<1)|A, where A,B are 0/1.
static inline int8_t decodeKnobDelta(uint8_t prevBA, uint8_t currBA) {
  if (prevBA == currBA) return 0;

  // Only count on transitions where A toggles (per brief)
  bool prevA = (prevBA & 0x01);
  bool currA = (currBA & 0x01);
  bool aToggled = (prevA != currA);
  if (!aToggled) {
    // intermediate state (A unchanged) -> no count
    return 0;
  }

  // Legal counted transitions (from the table):
  // 00 -> 01 : +1
  // 01 -> 00 : -1
  // 10 -> 11 : -1
  // 11 -> 10 : +1
  if (prevBA == 0b00 && currBA == 0b01) return +1;
  if (prevBA == 0b01 && currBA == 0b00) return -1;
  if (prevBA == 0b10 && currBA == 0b11) return -1;
  if (prevBA == 0b11 && currBA == 0b10) return +1;

  // Impossible transitions or anything else -> ignore for now
  return 0;
}

// ===== Audio ISR =====
void sampleISR() {
  static uint32_t phaseAcc = 0;

  uint32_t step = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  phaseAcc += step;

  int32_t Vout = (int32_t)(phaseAcc >> 24) - 128; // -128..127

  // Volume from knob3Rotation (0..8), log-ish taper by shifting
  uint8_t vol = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
  if (vol > 8) vol = 8; // safety clamp
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

  // Previous knob state {B,A}
  uint8_t prevKnob3BA = 0;

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    std::bitset<32> localInputs;

    // Scan rows 0..3 (row 3 needed for knob 3 A/B)
    for (uint8_t row = 0; row < 4; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        localInputs[row * 4 + col] = cols[col];
      }
    }

    // Decode knob 3: row 3 col0=A, col1=B
    // Inputs are active-low; convert to logical 0/1 where 1 means high.
    // For quadrature, treat "pressed/closed" vs not doesn't matter; we want logic level.
    bool A = (localInputs[3 * 4 + 0] != 0);
    bool B = (localInputs[3 * 4 + 1] != 0);
    uint8_t currKnob3BA = ((uint8_t)B << 1) | (uint8_t)A;

    int8_t delta = decodeKnobDelta(prevKnob3BA, currKnob3BA);
    prevKnob3BA = currKnob3BA;

    // Update knob rotation with limits 0..8
    uint8_t newRot = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
    if (delta > 0) {
      if (newRot < 8) newRot++;
    } else if (delta < 0) {
      if (newRot > 0) newRot--;
    }
    __atomic_store_n(&sysState.knob3Rotation, newRot, __ATOMIC_RELAXED);

    // Publish inputs to sysState (mutex protected)
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = localInputs;
    xSemaphoreGive(sysState.mutex);

    // Choose note (pressed = 0). Last key wins.
    uint32_t localStep = 0;
    for (int k = 0; k < 12; k++) {
      if (localInputs[k] == 0) localStep = stepSizes[k];
    }
    __atomic_store_n(&currentStepSize, localStep, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void *pvParameters) {
  (void)pvParameters;

  const TickType_t xFrequency = DISPLAY_PERIOD_MS / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Copy shared inputs under mutex (fast lock)
    std::bitset<32> inputsCopy;
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    inputsCopy = sysState.inputs;
    xSemaphoreGive(sysState.mutex);

    uint32_t key12 = inputsCopy.to_ulong() & 0xFFF;

    int selectedKey = -1;
    for (int k = 0; k < 12; k++) {
      if (inputsCopy[k] == 0) selectedKey = k;
    }

    uint8_t vol = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    u8g2.drawStr(0, 10, "Keys note vol");
    u8g2.setCursor(2, 22);
    u8g2.print(key12, HEX);

    u8g2.setCursor(52, 22);
    u8g2.print(selectedKey);

    u8g2.setCursor(80, 22);
    u8g2.print(vol);

    u8g2.sendBuffer();

    // Spec: LED toggle in display task
    digitalToggle(LED_BUILTIN);
  }
}

// ===== Setup / Loop =====
void setup() {
  // GPIO directions
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

  // Optional board routing enables (harmless if unused)
  setOutMuxBit(HKOE_BIT, HIGH);
  setOutMuxBit(HKOW_BIT, HIGH);

  Serial.begin(9600);
  Serial.println("Part 2 start");

  // Init shared state + mutex (must be before scheduler)
  sysState.inputs.reset();
  sysState.knob3Rotation = 8;                // start loud
  sysState.mutex = xSemaphoreCreateMutex();  // per brief

  // Audio setup
  analogWriteResolution(8);
  sampleTimer.setOverflow(AUDIO_FS_HZ, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  // Create tasks (scan higher priority than display)
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(scanKeysTask, "scanKeys", 64, NULL, 2, &scanKeysHandle);

  TaskHandle_t displayHandle = NULL;
  xTaskCreate(displayUpdateTask, "display", 256, NULL, 1, &displayHandle);

  // Start scheduler (must be last)
  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS: loop unused
}
