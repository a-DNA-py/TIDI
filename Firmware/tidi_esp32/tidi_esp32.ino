// TIDI car - ESP32 dev board (Hardware/wiring.svg)
// Drive + steering through the MDD3A, LiDAR head servos, TF-Luna, LED, buzzer, Bluetooth LE control.
//
// Advertises as "TIDI" over Bluetooth LE. A client writes a Cmd every ~100 ms and gets a Telem
// notification every 100 ms. No Cmd for LINK_TIMEOUT_MS -> motors off, buzzer off (azimuth, tilt and
// LED hold).
//
// Both head servos are positional MG90S (180 deg), set by angle: the Cmd's az (-90..90, 0 = the servo's
// centre) like its tilt. A lost link holds the last angle. Telem's headAz is the angle written.
//
// USB serial (115200) prints one status line every 500 ms.
// Tilt calibration: [ / ] step the tilt MG90S write() value by 1 and print it (the page's tilt is then
// ignored); r hands tilt back to the page. Use it to re-measure TILT_LEVEL_WRITE and TILT_45_WRITE.
//
// Room scan (a state machine, so BLE and the failsafe keep running):
// AZ_MIN -> AZ_MAX in SCAN_AZ_STEP steps, tilt 90 -> 0 and back at each azimuth, then back to the
// azimuth it started at. Drive, steering, azimuth and tilt from the page are ignored until it's back.
// Stop (or a lost link) steps it back; a second stop ends the scan and the head goes straight back.
// Started from the page, or over USB with s (q stops). USB prints START, az,tilt,dist_cm,strength per
// point, then END: Host/scan_room.py reads it, and its "small" column is this tilt angle (90 = level).
//
// Speak: the page turns text into Notes (a tone or glide, and an LED brightness) and writes them to
// SPEAK a few at a time; loop() plays them from a queue. While notes play they own the buzzer and LED.
// A dropped link empties the queue, like the buzzer.
//
// The Pi (TIDI's brain, GOFI's Node/tidi.py) talks over UART1 on GPIO16/17 with the same Cmd, ScanCtl
// and Speak payloads, framed (see the Pi link below), and gets Telem and every ScanPoint back. While its
// Cmd is fresh it drives and BLE's Cmd is ignored; either link counts for the failsafe.
//
// Wall stop: once the head is marked as facing forward (ScanCtl 2) and stays level and idle, forward
// drive is cut when the TF-Luna reads under WALL_STOP_CM, reads nothing it trusts up close (saturated,
// under 20 cm) or stops sending, and is ramped down to WALL_SLOW_CM. A weak reading (nothing in range)
// allows a crawl. Any manual azimuth turn forgets where forward is.

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <driver/gpio.h>

// ---------------- Pins (PROJECT.md -> Wiring - ESP32) ----------------
const int DRIVE_A = 25, DRIVE_B = 26;  // MDD3A M2A/M2B: drive motor
const int STEER_A = 32, STEER_B = 33;  // MDD3A M1A/M1B: steering motor
const int AZ_PIN = 18;                 // MG90S (180 deg): azimuth
const int SMALL_PIN = 19;              // MG90S: tilt
const int BATT_PIN = 34;               // ADC1
const int LIDAR_RX = 27;               // TF-Luna TX
const int LIDAR_TX = 4;                // UART2 needs a TX pin; nothing is connected to it
const int LED_PIN = 23;                // LED straight to GND, no resistor: pin set to its weakest drive, PWM for brightness
const int BUZZ_PIN = 22;               // passive buzzer, GPIO22 -> buzzer -> GND
const int PI_RX = 16;                  // UART1 RX <- Pi 5 GPIO14 (TXD0, header pin 8)
const int PI_TX = 17;                  // UART1 TX -> Pi 5 GPIO15 (RXD0, header pin 10)

// ---------------- Settings ----------------
// Azimuth angle: the servo's own, write() - 90, so 0 is its centre. The hosts draw + as anticlockwise
// seen from above; if the cloud comes out mirrored, set FLIP_AZ there.
const int AZ_MIN = -90, AZ_MAX = 90;   // deg: write() 0..180

// Tilt angle (the page's slider): 90 = line of sight level with the ground, 0 = straight up.
// Measured 2026-09-28 by eye: write 81 is level, write 40 is 45 deg up. Straight line through the two,
// so vertical is write -1: write 0 (the servo's end) stops about 1 deg short. Telem reports the
// angle actually reached.
const int TILT_LEVEL_WRITE = 81;       // write() value with the line of sight level
const int TILT_45_WRITE = 40;          // write() value at 45 deg up
const int TILT_MIN = 0, TILT_MAX = 90;

const float BATT_RATIO = (10.0 + 2.35) / 2.35;  // 10k top, 4.7k || 4.7k bottom
const float MOTOR_VOLTS = 7.0;         // average motor voltage at 100 % (duty = 7 V / battery)
const float CUTOFF_V = 9.9;            // 3S at 3.3 V/cell: motors off below this
const float RESUME_V = 10.5;           // and stay off until the pack reads above this

const int STEER_POWER = 100;           // % of the capped duty used for steering
const unsigned long STEER_MAX_MS = 3000;    // steering held longer than this is cut (MDD3A 5 A peak is < 5 s)
const unsigned long LINK_TIMEOUT_MS = 500;

const bool DRIVE_INVERT = false;       // flip if "forward" drives backwards
const bool STEER_INVERT = true;        // flip if "left" steers right

const uint32_t MOTOR_PWM_HZ = 20000;   // MDD3A takes PWM up to 20 kHz (Cytron)
const uint32_t SERVO_HZ = 50;
const uint8_t SERVO_BITS = 14;
const uint8_t BUZZ_BITS = 10;          // no other LEDC pin uses 10 bits, so the buzzer gets its own timer
                                       // and changing its tone can't retune the servos or motors
const int BUZZ_MIN_HZ = 100, BUZZ_MAX_HZ = 5000;
                                       // The LED uses the motors' MOTOR_PWM_HZ at 8 bits, so it shares their timer
const int SPEECH_MAX = 256;            // queued Notes; a longer text is cut off

// Wall stop, with the head facing forward. Set by estimate: the car's stopping distance at 7 V is unmeasured.
const int WALL_STOP_CM = 35;           // no forward drive under this
const int WALL_SLOW_CM = 90;           // forward power ramps from WALL_MIN_PCT at WALL_STOP_CM to 100 % here
const int WALL_MIN_PCT = 30;
const int WALL_WEAK_PCT = 40;          // most forward power while the reading is weak (< 100: nothing in range)
const int WATCH_TILT_MIN = 80;         // tilt angle at or above which the beam is taken to look ahead

// Scan
const int SCAN_AZ_STEP = 5;            // deg per azimuth step
const int SCAN_TILT_STEP = 5;          // deg per tilt step
const int SCAN_TILT_TOP = 0;           // tilt angle at the top of a column: straight up
const int SCAN_SAMPLES = 5;            // valid readings per point, median sent
const unsigned long AZ_SETTLE_MS = 300, TILT_FIRST_MS = 300, TILT_SETTLE_MS = 120, MEASURE_MAX_MS = 600;
const unsigned long AZ_TRAVEL_MS = 40; // per 5 deg step on the way to a column, not measuring. The MG90S is
                                       // rated 0.1 s / 60 deg at 4.8 V (datasheet, Hardware/BOM.md): ~8 ms
// ------------------------------------------

// Bluetooth LE service and characteristics
#define SERVICE_UUID "0535c297-fc42-4f0e-9508-79a22322ff5a"
#define CMD_UUID     "c5c17b93-91e5-4801-a5cb-fa27adfa7d73"
#define TELEM_UUID   "1c3bc823-5a4f-459a-8781-82b189409f20"
#define SCANCTL_UUID "a3c728ed-f2c9-410a-995b-ae95cbd1bd6d"   // write 1 byte: 1 = start, 0 = stop, 2 = the head faces forward now
#define POINT_UUID   "1160bfdb-3023-4df7-9156-6c73ae8af510"   // notify: one ScanPoint per point
#define SPEAK_UUID   "7e2d4b19-3c8a-4f61-b0d5-9a1e6c2f8b43"   // write: uint8 flags (bit0 = drop queued notes first) + 0..3 Notes

struct __attribute__((packed)) Cmd {   // page -> car
  int8_t drive;    // -100..100 %
  int8_t steer;    // -100 (left)..100 (right) %
  int8_t az;       // -90..90 deg, 0 = the servo's centre
  int8_t tilt;     // 0..90 deg, 90 = level
  uint8_t out;     // bit0 LED on, bit1 buzzer on
  uint16_t buzzHz; // buzzer frequency
};

struct __attribute__((packed)) Telem { // car -> page
  int16_t dist;    // cm, raw; bits 3-5 say why it isn't trusted. -1 = no frame yet
  uint16_t strength;
  uint16_t battMv;
  int8_t tilt;     // deg, as applied
  uint8_t flags;   // bit0 low-battery cutoff, bit1 steering timed out, bit2 LiDAR stale,
                   // bit3 weak signal (< 100), bit4 saturated (65535), bit5 under 20 cm, bit6 tilt limited
  uint8_t scan;    // 0 idle, 1 scanning, 2 returning (3 was the MG995's c / b turn, now gone)
  int16_t headAz;  // deg, as the Cmd's az, as written
  uint8_t ahead;   // bit0 head marked as facing forward, bit1 watching ahead (marked, level, idle),
                   // bit2 wall: forward cut, bit3 forward slowed, bit4 the Pi is driving, bit5 BLE connected
};

struct __attribute__((packed)) ScanPoint {  // car -> page, one per point
  uint16_t seq;    // 0 = first point of a scan
  int16_t az;      // deg, as headAz
  int8_t tilt;     // deg, 90 = level
  int16_t dist;    // cm, median of valid readings; -1 = none
  uint16_t strength;
};

struct __attribute__((packed)) Note {  // controller -> car, in a Speak
  uint16_t hz;     // at the start, 0 = silent
  uint16_t toHz;   // at the end: glides in a straight line from hz
  uint8_t dur;     // x 5 ms
  uint8_t led;     // LED brightness 0..255
};

Cmd cmd = {0, 0, 0, TILT_MAX, 0, 1000};      // from Bluetooth LE
Cmd piCmd = {0, 0, 0, TILT_MAX, 0, 1000};    // from the Pi, over UART1
Telem tel = {-1, 0, 0, TILT_MAX, 0, 0, 0, 0};
volatile unsigned long lastCmdMs = 0;
unsigned long lastPiCmdMs = 0;                // written and read in loop() only
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;   // onWrite runs in the BLE task

BLECharacteristic *telemChar, *pointChar;
bool connected = false;
volatile bool scanStartReq = false, scanStopReq = false;   // set in the BLE task, handled in loop()
volatile bool aheadReq = false;                            // ScanCtl 2: the head faces forward now
volatile bool linkLostReq = false;                         // a link dropped: a running scan goes home

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    if (c->getLength() != sizeof(Cmd)) return;
    portENTER_CRITICAL(&cmdMux);
    memcpy(&cmd, c->getData(), sizeof(Cmd));
    lastCmdMs = millis();
    portEXIT_CRITICAL(&cmdMux);
  }
};

void scanCtl(uint8_t v) {
  if (v == 2) aheadReq = true; else if (v) scanStartReq = true; else scanStopReq = true;
}

class ScanCtlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    if (c->getLength() != 1) return;
    scanCtl(c->getData()[0]);
  }
};

// Speech queue: a ring buffer, filled from BLE (its own task) or the Pi (loop()) and played in loop()
Note speech[SPEECH_MAX];
volatile uint16_t spIn = 0, spOut = 0;
volatile bool speechCut = false;       // also end the note playing now
portMUX_TYPE speechMux = portMUX_INITIALIZER_UNLOCKED;

// A Speak payload: uint8 flags (bit0 = drop queued notes first), then 0.. Notes
void queueNotes(const uint8_t *d, size_t n) {
  if (n < 1 || (n - 1) % sizeof(Note)) return;
  portENTER_CRITICAL(&speechMux);
  if (d[0] & 1) { spOut = spIn; speechCut = true; }
  for (size_t i = 1; i < n; i += sizeof(Note)) {
    uint16_t next = (spIn + 1) % SPEECH_MAX;
    if (next == spOut) break;                  // full: the rest is dropped
    memcpy(&speech[spIn], d + i, sizeof(Note));
    spIn = next;
  }
  portEXIT_CRITICAL(&speechMux);
}

class SpeakCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override { queueNotes(c->getData(), c->getLength()); }
};

// ---------------- Pi link: UART1, 115200 8N1 ----------------
// Frame: 0xA5, type, len, payload[len], CRC-8 (poly 0x07, init 0) over type, len and payload.
// Pi -> car: 'C' Cmd (7 bytes), 'S' ScanCtl (1), 'K' Speak (1 + 6 per Note, at most 42 Notes).
// Car -> Pi: 'T' Telem (12) every 100 ms, 'P' ScanPoint (9) per point.
// Must match GOFI's Node/tidi.py (frame, Frames).
const uint8_t PI_SYNC = 0xA5;
uint8_t piBuf[255];
int piState = 0, piLen = 0, piGot = 0;
uint8_t piType = 0;

uint8_t crc8(uint8_t crc, const uint8_t *d, size_t n) {
  while (n--) {
    crc ^= *d++;
    for (int i = 0; i < 8; i++) crc = crc & 0x80 ? (crc << 1) ^ 0x07 : crc << 1;
  }
  return crc;
}

void piSend(uint8_t type, const void *payload, uint8_t len) {
  uint8_t head[3] = {PI_SYNC, type, len};
  uint8_t crc = crc8(crc8(0, head + 1, 2), (const uint8_t *)payload, len);
  if (Serial1.availableForWrite() < len + 4) return;   // never block loop(): the Pi is behind, drop this one
  Serial1.write(head, 3);
  Serial1.write((const uint8_t *)payload, len);
  Serial1.write(crc);
}

void piFrame(uint8_t type, const uint8_t *d, int n) {
  if (type == 'C' && n == sizeof(Cmd)) { memcpy(&piCmd, d, sizeof(Cmd)); lastPiCmdMs = millis(); }
  else if (type == 'S' && n == 1) scanCtl(d[0]);
  else if (type == 'K') queueNotes(d, n);
}

void pollPi() {
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    if (piState == 0) { if (c == PI_SYNC) piState = 1; }
    else if (piState == 1) {                     // only a known type: a stray 0xA5 must not start a long wait
      if (c == 'C' || c == 'S' || c == 'K') { piType = c; piState = 2; }
      else piState = c == PI_SYNC ? 1 : 0;
    }
    else if (piState == 2) { piLen = c; piGot = 0; piState = piLen ? 3 : 4; }
    else if (piState == 3) { piBuf[piGot++] = c; if (piGot == piLen) piState = 4; }
    else {
      uint8_t head[2] = {piType, (uint8_t)piLen};
      if (crc8(crc8(0, head, 2), piBuf, piLen) == c) piFrame(piType, piBuf, piLen);
      piState = 0;                               // a bad CRC drops the frame; the next 0xA5 starts again
    }
  }
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { connected = true; }
  void onDisconnect(BLEServer *s) override { connected = false; linkLostReq = true; s->startAdvertising(); }
};

void onLidarFrame(int d, uint16_t s);

// TF-Luna: non-blocking frame parser, keeps the latest reading
uint8_t lbuf[9];
int lidx = 0;
unsigned long lastLidarMs = 0;
uint8_t lidarFlags = 0;                // bits 3-5 of Telem.flags

void pollLidar() {
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    if (lidx < 2) {
      if (c == 0x59) lbuf[lidx++] = c; else lidx = 0;
      continue;
    }
    lbuf[lidx++] = c;
    if (lidx == 9) {
      uint8_t sum = 0;
      for (int i = 0; i < 8; i++) sum += lbuf[i];
      if (sum == lbuf[8]) {
        int d = lbuf[2] | (lbuf[3] << 8);
        uint16_t s = lbuf[4] | (lbuf[5] << 8);
        // Raw distance goes out with the reason it fails the scan's validity rule, so the page
        // can show why instead of a blank
        tel.strength = s;
        tel.dist = d;
        lidarFlags = (s == 65535 ? 16 : s < 100 ? 8 : 0) | (d < 20 ? 32 : 0);
        lastLidarMs = millis();
        onLidarFrame(d, s);
      }
      lidx = 0;
    }
  }
}

// Battery: smoothed, with a latched cutoff
// Sampled every BATT_SAMPLE_MS, not every pass of loop(): smoothing per pass (thousands a second) smoothed nothing,
// so a motor starting dipped the reading under CUTOFF_V and the motors were cut mid-drive, five times in 17 s
// (2026-09-28, at 11.1 V resting). The cutoff now needs the smoothed voltage low for CUTOFF_HOLD_MS.
const unsigned long BATT_SAMPLE_MS = 20;
const float BATT_SMOOTH = 0.05;        // per sample: about 0.4 s to settle
const unsigned long CUTOFF_HOLD_MS = 2000;
float vbatt = 0;
bool lowBatt = true;                   // start off until a real reading says otherwise
unsigned long battAt = 0, battLowSince = 0;

void readBattery() {
  unsigned long now = millis();
  if (vbatt != 0 && now - battAt < BATT_SAMPLE_MS) return;
  battAt = now;
  float v = analogReadMilliVolts(BATT_PIN) / 1000.0 * BATT_RATIO;   // factory-calibrated mV
  vbatt = (vbatt == 0) ? v : vbatt * (1 - BATT_SMOOTH) + v * BATT_SMOOTH;
  if (vbatt >= CUTOFF_V) battLowSince = 0;
  else if (!battLowSince) battLowSince = now;
  if (battLowSince && now - battLowSince >= CUTOFF_HOLD_MS) lowBatt = true;
  else if (vbatt > RESUME_V) lowBatt = false;
  tel.battMv = (uint16_t)(vbatt * 1000);
}

// Duty at 100 %, so the motor averages MOTOR_VOLTS whatever the pack voltage
int dutyCap() {
  if (vbatt < 1) return 0;
  return min(255, (int)(255 * MOTOR_VOLTS / vbatt));
}

// MDD3A channel: forward is A=PWM, B=LOW; reverse is B=HIGH with A=PWM
// of the inverted duty, so the output alternates reverse/brake. Assumes each MDD3A output
// follows its input; unconfirmed.
void setMotor(int pinA, int pinB, int pct, bool invert) {
  if (invert) pct = -pct;
  int d = (long)abs(pct) * dutyCap() / 100;
  if (pct == 0 || d == 0) { digitalWrite(pinB, LOW); ledcWrite(pinA, 0); }
  else if (pct > 0)        { digitalWrite(pinB, LOW); ledcWrite(pinA, d); }
  else                     { digitalWrite(pinB, HIGH); ledcWrite(pinA, 255 - d); }
}

// Servo pulse from a write() angle, with the Arduino Servo library's map (0..180 -> 544..2400 us).
void servoWrite(int pin, int deg) {
  deg = constrain(deg, 0, 180);
  uint32_t us = 544 + (uint32_t)deg * (2400 - 544) / 180;
  ledcWrite(pin, us * ((1 << SERVO_BITS) - 1) / (1000000 / SERVO_HZ));
}

// Steering: cut after STEER_MAX_MS held in one direction; re-armed by centring or reversing
int steerDir = 0;
unsigned long steerSinceMs = 0;
bool steerTimedOut = false;

int steerOutput(int want) {
  int dir = (want > 0) - (want < 0);
  if (dir != steerDir) { steerDir = dir; steerSinceMs = millis(); steerTimedOut = false; }
  if (dir != 0 && millis() - steerSinceMs > STEER_MAX_MS) steerTimedOut = true;
  return steerTimedOut ? 0 : want;
}

// Azimuth: the servo's own angle, centred
int azToWrite(int deg) { return constrain(deg, AZ_MIN, AZ_MAX) + 90; }

int azWritten = -1;
int azCmd = 0;                         // deg the page or the Pi last asked for; held when the link drops

void writeAz(int w) {
  if (w != azWritten) { servoWrite(AZ_PIN, w); azWritten = w; }
}

// Degrees up from level -> write(), clamped to the servo's 0..180
int tiltToWrite(int deg) {
  float w = TILT_LEVEL_WRITE + (TILT_MAX - deg) * (TILT_45_WRITE - TILT_LEVEL_WRITE) / 45.0;
  return constrain((int)lround(w), 0, 180);
}

int writeToTilt(int w) {
  return (int)lround(TILT_MAX - (w - TILT_LEVEL_WRITE) * 45.0 / (TILT_45_WRITE - TILT_LEVEL_WRITE));
}

bool tiltLimited = false;
int tiltManual = -1;                   // write() value set over USB serial, -1 = page controls tilt
uint16_t buzzNow = 0;                  // Hz currently playing, 0 = silent
int tiltWritten = -1;
bool aheadKnown = false;               // the head was marked as facing forward and hasn't turned by hand since

// ---------------- Scan state machine ----------------
enum ScanState : uint8_t { SCAN_IDLE, SCAN_RUNNING, SCAN_RETURNING };
const int SCAN_TILTS = (TILT_MAX - SCAN_TILT_TOP) / SCAN_TILT_STEP + 1;   // 19 points per column
const int SCAN_COLS = (AZ_MAX - AZ_MIN) / SCAN_AZ_STEP + 1;              // -90..90: 37

ScanState scanState = SCAN_IDLE;
int headAz = 0;                        // deg the scan has the head at
int scanStartAz = 0;                   // where the scan goes back to
unsigned long settleUntilMs = 0;
int scanCol = 0, scanK = 0;            // column index, point index within the column
bool scanUp = true, sampling = false;
int scanTilt = TILT_MAX;               // tilt angle the scan wants now
int samples[SCAN_SAMPLES], nSamples = 0, nFrames = 0;
long strSum = 0;
uint16_t scanSeq = 0;

int colAz(int col) { return AZ_MIN + col * SCAN_AZ_STEP; }

void setScanTilt(int t) {
  scanTilt = t;
  settleUntilMs = millis() + (scanK == 0 ? TILT_FIRST_MS : TILT_SETTLE_MS);
  nSamples = 0; nFrames = 0; strSum = 0; sampling = true;
}

void onLidarFrame(int d, uint16_t s) {
  if (!sampling || millis() < settleUntilMs) return;
  nFrames++;
  if (s >= 100 && s != 65535 && d >= 20 && nSamples < SCAN_SAMPLES) { samples[nSamples++] = d; strSum += s; }
}

void sendPoint(int az, int tilt, int d, uint16_t s) {
  ScanPoint p = {scanSeq++, (int16_t)az, (int8_t)tilt, (int16_t)d, s};
  Serial.printf("%d,%d,%d,%u\n", az, tilt, d, s);
  if (connected) { pointChar->setValue((uint8_t *)&p, sizeof(p)); pointChar->notify(); }
  piSend('P', &p, sizeof(p));
}

void startScan() {
  if (scanState != SCAN_IDLE) return;
  scanState = SCAN_RUNNING;
  headAz = scanStartAz = azCmd;
  scanCol = 0; scanK = 0; scanUp = true; scanSeq = 0;
  sampling = false; scanTilt = TILT_MAX;         // measuring starts at the first column
  Serial.println("START");
}

void finishPoint() {
  sampling = false;
  int d = -1, s = 0;
  if (nSamples) {
    for (int i = 1; i < nSamples; i++) {         // insertion sort, median
      int v = samples[i], j = i - 1;
      while (j >= 0 && samples[j] > v) { samples[j + 1] = samples[j]; j--; }
      samples[j + 1] = v;
    }
    d = samples[nSamples / 2]; s = strSum / nSamples;
  }
  sendPoint(headAz, writeToTilt(tiltToWrite(scanTilt)), d, s);
  if (++scanK < SCAN_TILTS) {
    setScanTilt(scanUp ? TILT_MAX - scanK * SCAN_TILT_STEP : SCAN_TILT_TOP + scanK * SCAN_TILT_STEP);
    return;
  }
  scanK = 0; scanUp = !scanUp;                   // next column starts where this one ended
  if (++scanCol == SCAN_COLS) { scanState = SCAN_RETURNING; scanTilt = TILT_MAX; }
}

void updateScan() {
  bool lost = linkLostReq; linkLostReq = false;
  if (scanStopReq || lost) {
    // Stop during a scan: step back to where it started. Stop again: end it, and the head goes straight back.
    if (scanState == SCAN_RUNNING) {
      scanState = SCAN_RETURNING; sampling = false; scanTilt = TILT_MAX; Serial.println("STOPPED, going back");
    } else if (scanStopReq && scanState != SCAN_IDLE) {
      scanState = SCAN_IDLE; sampling = false; Serial.println("HALTED");
    }
    scanStopReq = false;
  }
  if (aheadReq) {                                // only while idle: the head is mid-sweep during a scan
    aheadReq = false;
    if (scanState == SCAN_IDLE) { aheadKnown = true; Serial.println("AHEAD: the head faces forward"); }
  }
  if (scanStartReq) { scanStartReq = false; startScan(); }
  if (scanState == SCAN_IDLE) return;

  unsigned long now = millis();
  if ((long)(now - settleUntilMs) < 0) return;

  int target = scanState == SCAN_RUNNING ? colAz(scanCol) : scanStartAz;
  if (headAz != target) {                        // one step towards it; settle longer on arriving
    headAz += constrain(target - headAz, -SCAN_AZ_STEP, SCAN_AZ_STEP);
    writeAz(azToWrite(headAz));
    settleUntilMs = now + (headAz == target ? AZ_SETTLE_MS : AZ_TRAVEL_MS);
    return;
  }

  if (scanState == SCAN_RUNNING) {
    if (!sampling) setScanTilt(scanUp ? TILT_MAX : SCAN_TILT_TOP);   // arrived at a new column
    else if (nSamples == SCAN_SAMPLES || nFrames >= SCAN_SAMPLES * 3 || now - settleUntilMs > MEASURE_MAX_MS) finishPoint();
    return;
  }
  Serial.println("END");
  scanState = SCAN_IDLE;
}

// Speech player: the next Note starts when the last one's time is up
Note noteNow;
bool noteOn = false;
unsigned long noteStartMs = 0;
int speechHz = -1, speechLed = 0;      // what the note wants now; -1 = not speaking
int ledNow = -1;                       // brightness written, 0..255

void updateSpeech(bool linkOk) {
  unsigned long now = millis();
  if (!linkOk) { portENTER_CRITICAL(&speechMux); spOut = spIn; portEXIT_CRITICAL(&speechMux); }
  if (speechCut || !linkOk) { speechCut = false; noteOn = false; }
  unsigned long span = max(1, (int)noteNow.dur) * 5UL;
  if (noteOn && now - noteStartMs >= span) noteOn = false;
  if (!noteOn) {
    portENTER_CRITICAL(&speechMux);
    bool have = spOut != spIn;
    if (have) { noteNow = speech[spOut]; spOut = (spOut + 1) % SPEECH_MAX; }
    portEXIT_CRITICAL(&speechMux);
    if (!have) { speechHz = -1; return; }
    noteOn = true; noteStartMs = now;
    span = max(1, (int)noteNow.dur) * 5UL;
  }
  unsigned long t = (now - noteStartMs) / 5 * 5;   // glide in 5 ms steps, so the buzzer isn't retuned every pass of loop()
  speechHz = noteNow.hz ? noteNow.hz + ((long)noteNow.toHz - noteNow.hz) * (long)t / (long)span : 0;
  speechLed = noteNow.led;
}

// The most forward drive (%) the TF-Luna allows while it watches ahead; 100 when it isn't watching.
int forwardLimit(bool watching) {
  if (!watching) return 100;
  if ((lidarFlags & (16 | 32)) || millis() - lastLidarMs > 500) return 0;   // saturated, under 20 cm, or silent
  if (lidarFlags & 8) return WALL_WEAK_PCT;
  if (tel.dist < WALL_STOP_CM) return 0;
  if (tel.dist >= WALL_SLOW_CM) return 100;
  return WALL_MIN_PCT + (long)(100 - WALL_MIN_PCT) * (tel.dist - WALL_STOP_CM) / (WALL_SLOW_CM - WALL_STOP_CM);
}

bool piDriving = false, wallCut = false, wallSlowed = false;

void applyCmd() {
  portENTER_CRITICAL(&cmdMux);
  Cmd c = cmd;
  bool bleOk = lastCmdMs && millis() - lastCmdMs < LINK_TIMEOUT_MS;
  portEXIT_CRITICAL(&cmdMux);
  piDriving = lastPiCmdMs && millis() - lastPiCmdMs < LINK_TIMEOUT_MS;
  if (piDriving) c = piCmd;                        // the Pi relays the Mac's controls too, so it wins
  bool linkOk = piDriving || bleOk;
  static bool piWas = false;
  if (piWas && !piDriving) linkLostReq = true;     // the Pi went quiet: a running scan goes home
  piWas = piDriving;

  int drive = linkOk ? c.drive : 0;
  int steer = linkOk ? steerOutput(c.steer) : steerOutput(0);
  int az = linkOk ? constrain(c.az, AZ_MIN, AZ_MAX) : azCmd;   // a lost link holds the head where it is
  bool scanning = scanState != SCAN_IDLE;          // the scan owns the head, and the car stays put
  if (lowBatt || scanning) { drive = 0; steer = 0; }
  if (!scanning && az != azCmd) aheadKnown = false;   // turned by hand: forward is no longer known
  if (!scanning) azCmd = az;

  int limit = forwardLimit(aheadKnown && !scanning && tel.tilt >= WATCH_TILT_MIN);
  wallCut = drive > 0 && limit == 0;
  wallSlowed = drive > limit && limit > 0;
  drive = min(drive, limit);

  setMotor(DRIVE_A, DRIVE_B, drive, DRIVE_INVERT);
  setMotor(STEER_A, STEER_B, (long)steer * STEER_POWER / 100, STEER_INVERT);
  if (!scanning) writeAz(azToWrite(az));

  int t = scanning ? scanTilt : constrain(c.tilt, TILT_MIN, TILT_MAX);
  int w = tiltManual >= 0 && !scanning ? tiltManual : tiltToWrite(t);
  if (w != tiltWritten) { servoWrite(SMALL_PIN, w); tiltWritten = w; }
  tel.tilt = writeToTilt(w);
  tiltLimited = tiltManual < 0 && tel.tilt != t;

  updateSpeech(linkOk);
  int led = speechHz >= 0 ? speechLed : (c.out & 1) ? 255 : 0;
  if (led != ledNow) { ledcWrite(LED_PIN, (uint32_t)led * led / 255); ledNow = led; }   // squared: steps look more even to the eye
  uint16_t hz = speechHz > 0 ? constrain(speechHz, BUZZ_MIN_HZ, BUZZ_MAX_HZ) : speechHz == 0 ? 0
              : (linkOk && (c.out & 2)) ? constrain(c.buzzHz, BUZZ_MIN_HZ, BUZZ_MAX_HZ) : 0;
  if (hz != buzzNow) { ledcWriteTone(BUZZ_PIN, hz); buzzNow = hz; }
}

void handleSerial() {
  if (!Serial.available()) return;
  char ch = Serial.read();
  if (ch == '[' || ch == ']') {
    if (tiltManual < 0) tiltManual = tiltWritten;
    tiltManual = constrain(tiltManual + (ch == ']' ? 1 : -1), 0, 180);
    Serial.print("tilt write="); Serial.println(tiltManual);
  } else if (ch == 'r') {
    tiltManual = -1;
    Serial.println("tilt back to the page");
  } else if (ch == 's') {
    scanStartReq = true;
  } else if (ch == 'q') {
    scanStopReq = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.setRxBufferSize(512);
  Serial2.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  Serial1.setRxBufferSize(1024);                 // a Speak frame is up to 258 bytes
  Serial1.begin(115200, SERIAL_8N1, PI_RX, PI_TX);

  pinMode(DRIVE_B, OUTPUT); pinMode(STEER_B, OUTPUT);
  ledcAttach(DRIVE_A, MOTOR_PWM_HZ, 8);
  ledcAttach(STEER_A, MOTOR_PWM_HZ, 8);
  setMotor(DRIVE_A, DRIVE_B, 0, false);
  setMotor(STEER_A, STEER_B, 0, false);

  ledcAttach(LED_PIN, MOTOR_PWM_HZ, 8);
  ledcWrite(LED_PIN, 0);
  gpio_set_drive_capability((gpio_num_t)LED_PIN, GPIO_DRIVE_CAP_0);   // after the attach; the pin's driver is the only current limit
  ledcAttach(BUZZ_PIN, 1000, BUZZ_BITS);
  ledcWriteTone(BUZZ_PIN, 0);

  ledcAttach(AZ_PIN, SERVO_HZ, SERVO_BITS);      // the head turns to the centre at power-up
  writeAz(azToWrite(0));
  ledcAttach(SMALL_PIN, SERVO_HZ, SERVO_BITS);
  tiltWritten = tiltToWrite(TILT_MAX);
  servoWrite(SMALL_PIN, tiltWritten);

  BLEDevice::init("TIDI");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *svc = server->createService(SERVICE_UUID);
  BLECharacteristic *cmdChar = svc->createCharacteristic(CMD_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  cmdChar->setCallbacks(new CmdCallbacks());
  telemChar = svc->createCharacteristic(TELEM_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  telemChar->addDescriptor(new BLE2902());
  BLECharacteristic *scanCtlChar = svc->createCharacteristic(SCANCTL_UUID, BLECharacteristic::PROPERTY_WRITE);
  scanCtlChar->setCallbacks(new ScanCtlCallbacks());
  pointChar = svc->createCharacteristic(POINT_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pointChar->addDescriptor(new BLE2902());
  BLECharacteristic *speakChar = svc->createCharacteristic(SPEAK_UUID, BLECharacteristic::PROPERTY_WRITE);
  speakChar->setCallbacks(new SpeakCallbacks());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("TIDI ESP32 ready, advertising as TIDI");
}

void loop() {
  pollLidar();
  pollPi();
  readBattery();
  handleSerial();
  updateScan();
  applyCmd();
  tel.flags = (lowBatt ? 1 : 0) | (steerTimedOut ? 2 : 0) | (millis() - lastLidarMs > 500 ? 4 : 0)
            | lidarFlags | (tiltLimited ? 64 : 0);
  tel.scan = scanState;
  tel.headAz = azWritten - 90;
  bool watching = aheadKnown && scanState == SCAN_IDLE && tel.tilt >= WATCH_TILT_MIN;
  tel.ahead = (aheadKnown ? 1 : 0) | (watching ? 2 : 0) | (wallCut ? 4 : 0) | (wallSlowed ? 8 : 0)
            | (piDriving ? 16 : 0) | (connected ? 32 : 0);

  static unsigned long lastTelem = 0;
  if (millis() - lastTelem >= 100) {
    lastTelem = millis();
    if (connected) { telemChar->setValue((uint8_t *)&tel, sizeof(tel)); telemChar->notify(); }
    piSend('T', &tel, sizeof(tel));
  }

  static unsigned long lastPrint = 0;
  if (scanState == SCAN_IDLE && millis() - lastPrint >= 500) {   // quiet during a scan: scan_room.py reads the lines
    lastPrint = millis();
    Serial.print("ble="); Serial.print(connected);
    Serial.print(" link="); Serial.print(lastCmdMs && millis() - lastCmdMs < LINK_TIMEOUT_MS);
    Serial.print(" drive="); Serial.print(cmd.drive);
    Serial.print(" steer="); Serial.print(cmd.steer);
    Serial.print(" az="); Serial.print(cmd.az);
    Serial.print(" head="); Serial.print(tel.headAz);
    Serial.print(" azw="); Serial.print(azWritten);
    Serial.print(" tilt="); Serial.print(tel.tilt);
    Serial.print(" write="); Serial.print(tiltWritten);
    Serial.print(" led="); Serial.print(ledNow);
    Serial.print(" buzz="); Serial.print(buzzNow);
    Serial.print(" dist="); Serial.print(tel.dist);
    Serial.print(" batt="); Serial.print(vbatt, 2);
    Serial.print(" pi="); Serial.print(piDriving);
    Serial.print(" ahead="); Serial.print(tel.ahead);
    Serial.print(" flags="); Serial.println(tel.flags);
  }
}
