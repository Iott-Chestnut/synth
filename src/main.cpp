// main.cpp — Section 4 (FreeRTOS threading) FULL FILE
// - scanKeysTask @ 50ms (priority 2)
// - displayUpdateTask @ 100ms (priority 1, toggles LED per spec)
// - sampleISR @ 22kHz timer (TIM1) generates sawtooth audio
//
// NOTE: I’ve included a mutex for sysState.inputs so you don’t hit the
// “possible synchronisation bug” the brief mentions.

#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>

//Constants
const uint32_t intervalMs = 100; // display task interval (ms)

//Pin definitions
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

//Output multiplexer bits
const int KNOB_MODE = 2;
const int DEN_BIT   = 3;
const int DRST_BIT  = 4;
const int HKOW_BIT  = 5;
const int HKOE_BIT  = 6;

//Display
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// Shared system state (per brief)
struct {
  std::bitset<32> inputs;
} sysState;

// Mutex to protect sysState.inputs
SemaphoreHandle_t sysStateMutex = nullptr;

// Audio globals
volatile uint32_t currentStepSize = 0;

// Timer for 22 kHz sampling (per brief)
HardwareTimer sampleTimer(TIM1);

// 12 notes (C..B), equal temperament, A4=440Hz is index 9, fs=22kHz
// S = (2^32 * f)/fs rounded
const uint32_t stepSizes[12] = {
  51076057, 54113197, 57330935, 60740010,
  64351799, 68178356, 72232452, 76527617,
  81078186, 85899346, 91007187, 96418756
};

// ---------- OUT MUX (starter code) ----------
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

// ---------- Matrix helpers ----------
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

// ---------- Audio ISR ----------
void sampleISR() {
  static uint32_t phaseAcc = 0;

  // Read step size (ISR cannot be interrupted by same priority, so plain read is fine)
  uint32_t step = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);

  phaseAcc += step;

  int32_t Vout = (int32_t)(phaseAcc >> 24) - 128; // -128..127
  uint8_t out = (uint8_t)(Vout + 128);            // 0..255

  // Drive both channels
  analogWrite(OUTR_PIN, out);
  analogWrite(OUTL_PIN, out);
}

// ---------- FreeRTOS Tasks ----------
void scanKeysTask(void *pvParameters) {
  (void)pvParameters;

  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    std::bitset<32> localInputs;

    for (uint8_t row = 0; row < 3; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        localInputs[row * 4 + col] = cols[col];
      }
    }

    // Publish inputs (mutex-protected)
    if (sysStateMutex) {
      xSemaphoreTake(sysStateMutex, portMAX_DELAY);
      sysState.inputs = localInputs;
      xSemaphoreGive(sysStateMutex);
    } else {
      sysState.inputs = localInputs;
    }

    // Choose note (pressed = 0). Last pressed in scan order wins.
    uint32_t localStep = 0;
    for (int k = 0; k < 12; k++) {
      if (localInputs[k] == 0) localStep = stepSizes[k];
    }

    __atomic_store_n(&currentStepSize, localStep, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void *pvParameters) {
  (void)pvParameters;

  const TickType_t xFrequency = intervalMs / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    std::bitset<32> inputsCopy;
    if (sysStateMutex) {
      xSemaphoreTake(sysStateMutex, portMAX_DELAY);
      inputsCopy = sysState.inputs;
      xSemaphoreGive(sysStateMutex);
    } else {
      inputsCopy = sysState.inputs;
    }

    uint32_t key12 = inputsCopy.to_ulong() & 0xFFF;

    int selectedKey = -1;
    for (int k = 0; k < 12; k++) {
      if (inputsCopy[k] == 0) selectedKey = k;
    }

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "Keys / note");
    u8g2.setCursor(2, 22);
    u8g2.print(key12, HEX);
    u8g2.setCursor(52, 22);
    u8g2.print(selectedKey);
    u8g2.sendBuffer();

    // Spec: toggle LED in display task
    digitalToggle(LED_BUILTIN);
  }
}

// ---------- Setup / Loop ----------
void setup() {
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
  Serial.println("Hello World");

  // Audio output config
  analogWriteResolution(8);

  // Start audio timer ISR (22kHz)
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  // Create mutex
  sysStateMutex = xSemaphoreCreateMutex();

  // Create tasks (per brief stack sizes + priorities)
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(scanKeysTask, "scanKeys", 64, NULL, 2, &scanKeysHandle);

  TaskHandle_t displayHandle = NULL;
  xTaskCreate(displayUpdateTask, "display", 256, NULL, 1, &displayHandle);

  // MUST start scheduler if FreeRTOS is a dependency
  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS systems leave loop empty
}
