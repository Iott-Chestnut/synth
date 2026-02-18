#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>

//Constants
const uint32_t interval = 100; //Display update interval

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

//Function to set outputs using key matrix
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

/* =========================
   PART 2: READ INPUTS
   ========================= */

// Read the 4 column inputs and return as bitset<4>.
// result[0]=C0, result[1]=C1, result[2]=C2, result[3]=C3
std::bitset<4> readCols() {
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

// Select a given row of the switch matrix (glitch-free).
// Disable REN before changing address bits, then enable afterwards.
void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);

  digitalWrite(RA0_PIN, (rowIdx & 0x01) ? HIGH : LOW);
  digitalWrite(RA1_PIN, (rowIdx & 0x02) ? HIGH : LOW);
  digitalWrite(RA2_PIN, (rowIdx & 0x04) ? HIGH : LOW);

  digitalWrite(REN_PIN, HIGH);
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

  // Columns: recommend pullups for active-low keys (pressed = 0)
  pinMode(C0_PIN, INPUT_PULLUP);
  pinMode(C1_PIN, INPUT_PULLUP);
  pinMode(C2_PIN, INPUT_PULLUP);
  pinMode(C3_PIN, INPUT_PULLUP);

  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);    //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);   //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);    //Enable display power supply
  setOutMuxBit(KNOB_MODE, HIGH);  //Read knobs through key matrix (leave as starter)

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}

void loop() {
  static uint32_t next = millis();

  while (millis() < next) {}  //Wait for next interval
  next += interval;

  // ----- Scan keys (rows 0..2 = 12 music keys) -----
  std::bitset<32> inputs;

  for (uint8_t row = 0; row < 3; row++) {
    setRow(row);
    delayMicroseconds(3);          // settle time
    std::bitset<4> cols = readCols();

    // Copy 4 bits into the appropriate slice
    for (uint8_t col = 0; col < 4; col++) {
      inputs[row * 4 + col] = cols[col];
    }
  }

  // 12 key bits are inputs[0..11]
  uint32_t key12 = inputs.to_ulong() & 0xFFF;

  // ----- Update display -----
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  u8g2.drawStr(0, 10, "Keys (12-bit hex):");
  u8g2.setCursor(2, 28);
  u8g2.print(key12, HEX);

  u8g2.sendBuffer();

  //Toggle LED (keep this as per spec)
  digitalToggle(LED_BUILTIN);
}
