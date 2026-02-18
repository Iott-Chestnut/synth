#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>

//Constants
const uint32_t interval = 100; //Display update interval (OLED + LED)

//Pin definitions
//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

//Output multiplexer bits
const int KNOB_MODE = 2;
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// Global: accessed by ISR + main loop
volatile uint32_t currentStepSize = 0;

// Debug: ISR call counter
volatile uint32_t isrCount = 0;

// Timer per brief (TIM1)
HardwareTimer sampleTimer(TIM1);


const uint32_t stepSizes[12] = {
  51076057, 54113197, 57330935, 60740010,
  64351799, 68178356, 72232452, 76527617,
  81078186, 85899346, 91007187, 96418756
};

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
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);

  digitalWrite(RA0_PIN, (rowIdx & 0x01) ? HIGH : LOW);
  digitalWrite(RA1_PIN, (rowIdx & 0x02) ? HIGH : LOW);
  digitalWrite(RA2_PIN, (rowIdx & 0x04) ? HIGH : LOW);

  digitalWrite(REN_PIN, HIGH);
}



void sampleISR() {
  static uint32_t phaseAcc = 0;

  // Debug count
  isrCount++;

  static uint32_t div = 0;
  div++;
  if (div >= 11000) {
    div = 0;
    digitalToggle(LED_BUILTIN);
  }

  phaseAcc += currentStepSize;

  int32_t Vout = (int32_t)(phaseAcc >> 24) - 128;   // -128..127

  analogWrite(OUTR_PIN, (uint8_t)(Vout + 128));

}

void setup() {
  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Columns: keys are active-low; pullups are the safe default
  pinMode(C0_PIN, INPUT_PULLUP);
  pinMode(C1_PIN, INPUT_PULLUP);
  pinMode(C2_PIN, INPUT_PULLUP);
  pinMode(C3_PIN, INPUT_PULLUP);

  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display (starter code)
  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);
  setOutMuxBit(KNOB_MODE, HIGH);

  //UART
  Serial.begin(9600);
  Serial.println("Hello World");

  analogWriteResolution(8);

  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
}

void loop() {
  static uint32_t next = millis();

  while (millis() < next) {}
  next += interval;

  std::bitset<32> inputs;

  for (uint8_t row = 0; row < 3; row++) {
    setRow(row);
    delayMicroseconds(3);
    std::bitset<4> cols = readCols();
    for (uint8_t col = 0; col < 4; col++) {
      inputs[row * 4 + col] = cols[col];
    }
  }

  uint32_t localStep = 0;
  int selectedKey = -1;

  for (int k = 0; k < 12; k++) {
    if (inputs[k] == 0) {
      localStep = stepSizes[k];
      selectedKey = k;
    }
  }

  __atomic_store_n(&currentStepSize, localStep, __ATOMIC_RELAXED);

  static uint32_t lastIsr = 0;
  uint32_t nowIsr = isrCount;
  uint32_t deltaIsr = nowIsr - lastIsr;
  lastIsr = nowIsr;
  uint32_t isrPerSec = deltaIsr * 10; // interval 100ms

  uint32_t key12 = inputs.to_ulong() & 0xFFF;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, "Keys / note / ISR/s");

  u8g2.setCursor(2, 22);
  u8g2.print(key12, HEX);

  u8g2.setCursor(52, 22);
  u8g2.print(selectedKey);

  u8g2.setCursor(80, 22);
  u8g2.print(isrPerSec);

  u8g2.sendBuffer();

  // NOTE: LED is now being toggled by ISR for debug
  // so we do NOT toggle it here.
}
