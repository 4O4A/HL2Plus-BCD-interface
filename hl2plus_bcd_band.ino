/*
  HL2+ (Elecraft/KX3-style CAT) -> Yaesu BCD band data (+ Yaesu analog band voltage)
  Arduino Nano (ATmega328P, 16 MHz)

  Wiring
    D2        <- J4/DE-9 pin 2 (ExtAMPTxD), SoftwareSerial RX
    5V        -> J4/DE-9 pin 1 (pull-up supply for the open-drain serial line)
    GND       <-> J4/DE-9 pin 5, amplifier GND
    D4 D5 D6 D7 -> BCD band data A B C D (5 V TTL, A = LSB)
    D9        -> 1 kOhm -> (10 uF to GND) -> Yaesu analog BAND input (optional)
    D13          built-in LED, flashes on each valid frequency frame
    USB          115200 baud debug output

  BCD code (Yaesu FT-1000/FT-2000/FTdx, Elecraft K3, Acom, Expert ...), D C B A:
    160m 0001  80m 0010  40m 0011  30m 0100  20m 0101
     17m 0110  15m 0111  12m 1000  10m 1001   6m 1010   none 0000
*/

#include <SoftwareSerial.h>

// ---------- user settings ----------
const long    RADIO_BAUD   = 38400;  // KX3 / HR-50 default; try 9600 or 4800 if nothing decodes
const bool    RADIO_INVERT = false;  // set true if the serial line is inverted (garbage at every baud rate)
#define       POLL_RADIO     0       // 1 = Nano sends "FA;" every 250 ms (only if D3 is wired to a radio RX input)
const float   VCC_VOLTS    = 5.00;   // measured Nano 5V rail, for the analog output
const bool    DEBUG        = true;

const uint8_t PIN_RX  = 2;
const uint8_t PIN_TX  = 3;
const uint8_t PIN_BCD[4] = { 4, 5, 6, 7 };   // A, B, C, D
const uint8_t PIN_PWM = 9;                   // must stay D9 (Timer1 OC1A)
const uint8_t PIN_LED = 13;

struct Band { uint32_t lo_khz; uint32_t hi_khz; uint8_t bcd; float volts; const char* name; };
const Band BANDS[] = {
  {  1500,  2500, 0b0001, 0.33, "160m" },
  {  3000,  4500, 0b0010, 0.67, "80m"  },
  {  5000,  5600, 0b0011, 1.00, "60m (40m code)" },  // no Yaesu code for 60m; change to 0b0010/0.67 for 80m LPF
  {  6500,  7500, 0b0011, 1.00, "40m"  },
  {  9500, 10500, 0b0100, 1.33, "30m"  },
  { 13500, 14500, 0b0101, 1.67, "20m"  },
  { 17500, 18500, 0b0110, 2.00, "17m"  },
  { 20500, 21500, 0b0111, 2.33, "15m"  },
  { 24000, 25500, 0b1000, 2.67, "12m"  },
  { 27500, 30000, 0b1001, 3.00, "10m"  },
  { 49000, 55000, 0b1010, 3.33, "6m"   },
};
const uint8_t N_BANDS = sizeof(BANDS) / sizeof(BANDS[0]);
// -----------------------------------

SoftwareSerial radio(PIN_RX, PIN_TX, RADIO_INVERT);

char     frame[40];
uint8_t  frameLen = 0;
int8_t   lastBand = -2;      // -1 = out of band, -2 = nothing yet
uint32_t ledOffAt = 0;
uint32_t lastPoll = 0;

void setupPwm() {
  pinMode(PIN_PWM, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);   // fast PWM 10-bit, 15.6 kHz on D9
  TCCR1B = _BV(WGM12)  | _BV(CS10);
  OCR1A  = 0;
}

void setVolts(float v) {
  if (v < 0) v = 0;
  if (v > VCC_VOLTS) v = VCC_VOLTS;
  OCR1A = (uint16_t)(v / VCC_VOLTS * 1023.0 + 0.5);
}

void setBcd(uint8_t code) {
  for (uint8_t i = 0; i < 4; i++) digitalWrite(PIN_BCD[i], (code >> i) & 1);
}

void applyFrequency(uint32_t hz) {
  uint32_t khz = hz / 1000UL;
  int8_t idx = -1;
  for (uint8_t i = 0; i < N_BANDS; i++) {
    if (khz >= BANDS[i].lo_khz && khz <= BANDS[i].hi_khz) { idx = i; break; }
  }
  if (idx == lastBand) return;
  lastBand = idx;

  uint8_t code = 0; float v = 0.0; const char* name = "out of band";
  if (idx >= 0) { code = BANDS[idx].bcd; v = BANDS[idx].volts; name = BANDS[idx].name; }
  setBcd(code);
  setVolts(v);

  if (DEBUG) {
    Serial.print(F("f=")); Serial.print(hz); Serial.print(F(" Hz  ")); Serial.print(name);
    Serial.print(F("  BCD DCBA="));
    for (int8_t b = 3; b >= 0; b--) Serial.print((code >> b) & 1);
    Serial.print(F("  analog=")); Serial.print(v, 2); Serial.println(F(" V"));
  }
}

// Elecraft frames: "FA00014074000;" (11-digit Hz) or "IF00014074000     +000000 ...;"
void handleFrame(const char* f, uint8_t n) {
  bool fa  = (f[0] == 'F' && f[1] == 'A');
  bool iff = (f[0] == 'I' && f[1] == 'F');
  if (!(fa || iff) || n < 13) return;
  if (f[2] != '0' || f[3] != '0') return;
  uint32_t hz = 0;
  for (uint8_t i = 4; i < 13; i++) {
    if (f[i] < '0' || f[i] > '9') return;
    hz = hz * 10UL + (f[i] - '0');
  }
  if (hz < 100000UL || hz > 60000000UL) return;
  digitalWrite(PIN_LED, HIGH);
  ledOffAt = millis() + 60;
  applyFrequency(hz);
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  for (uint8_t i = 0; i < 4; i++) { pinMode(PIN_BCD[i], OUTPUT); digitalWrite(PIN_BCD[i], LOW); }
  setupPwm();
  Serial.begin(115200);
  radio.begin(RADIO_BAUD);
  if (DEBUG) Serial.println(F("HL2+ CAT -> BCD band data. Waiting for FA/IF frames..."));
}

void loop() {
  while (radio.available()) {
    char c = radio.read();
    if (c == '\r' || c == '\n') continue;
    if (c == ';') {
      frame[frameLen] = 0;
      handleFrame(frame, frameLen);
      frameLen = 0;
    } else if (frameLen < sizeof(frame) - 1) {
      frame[frameLen++] = c;
    } else {
      frameLen = 0;
    }
  }

#if POLL_RADIO
  if (millis() - lastPoll > 250) { lastPoll = millis(); radio.print(F("FA;")); }
#endif

  if (ledOffAt && (int32_t)(millis() - ledOffAt) >= 0) { digitalWrite(PIN_LED, LOW); ledOffAt = 0; }
}
