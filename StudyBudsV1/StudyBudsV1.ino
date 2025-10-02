#include <Servo.h>

const uint8_t SERVO_PINS[3] = {2, 3, 4};    // servos that move the leaves
const uint8_t BTN_PINS[3]   = {13, 12, 11}; // hidden break-vote inputs (active LOW)

struct SonarPins { uint8_t trig; uint8_t echo; };
const SonarPins SONARS[3] = {
  {10, 9},  // leaf 1 sensor: trig, echo
  { 8, 7},  // leaf 2 sensor: trig, echo
  { 6, 5}   // leaf 3 sensor: trig, echo
};

const float    TRIP_CM     = 10.0;        // leaves cover items once all lie within 10 cm from ultrasonic sensor
const uint32_t FOCUS_TIME  = 300000UL;     // leaves stay closed for up to 1 min of focus
const uint32_t BREAK_TIME  = 60000UL;     // leaves stay open 10 s before next planting
const uint32_t PRE_CLOSE_DELAY_MS = 1000; // grace period from all items detected to leaves closing

// Servo angles (tune for gentle lift/close motion; per leaf customization optional)
const int SERVO_OPEN_POS   = 90;  // leaves lift and release items here
const int SERVO_CLOSED_POS = 10;  // leaves cover items

// Motion tuning
const uint8_t CLOSE_STEP_DEG   = 2;   // smaller step -> smoother, slower close
const uint16_t CLOSE_DWELL_MS  = 20;  // dwell between steps while closing
const uint8_t OPEN_STEP_DEG    = 2;   // smaller step -> smoother, slower open
const uint16_t OPEN_DWELL_MS   = 25;  // linger slightly longer between steps while opening

// Sonar timing
const unsigned long PULSE_TIMEOUT_US = 25000UL; // keep pulseIn responsive
const uint16_t INTER_PING_DELAY_MS   = 30;      // stagger leaves to reduce ultrasonic crosstalk

enum SystemState { OPEN, CLOSED_WAIT };
SystemState state = OPEN;

Servo servos[3];

// Vote latches (reset each time the leaves lower for the focus session)
bool votePressed[3] = {false, false, false};

// Debounce/sample cadence
uint32_t lastBtnSampleMs = 0;
const uint16_t BTN_SAMPLE_MS = 10;

uint32_t closedSinceMs = 0;
uint32_t refractoryUntilMs = 0; // short shared break; ignore replanting until this time
bool closingDelayArmed = false;
uint32_t closingReadyAtMs = 0;

float readDistanceCm(const SonarPins& p) {
  // Trigger pulse
  pinMode(p.trig, OUTPUT);
  digitalWrite(p.trig, LOW);
  delayMicroseconds(2);
  digitalWrite(p.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(p.trig, LOW);

  // Listen for echo
  pinMode(p.echo, INPUT);
  unsigned long duration = pulseIn(p.echo, HIGH, PULSE_TIMEOUT_US);
  if (duration == 0) return 9999.0f; // timeout -> no item planted over this leaf
  return duration / 58.0f;           // ~58 us per cm (round trip)
}

inline bool allWithinThreshold(float a, float b, float c, float th) {
  return (a <= th) && (b <= th) && (c <= th);
}

inline bool allTrue(const bool arr[3]) {
  return arr[0] && arr[1] && arr[2];
}

inline void writeAllServos(int pos) {
  for (int i = 0; i < 3; ++i) servos[i].write(pos);
}

void moveAllServosSmooth(int target, uint8_t stepDeg, uint16_t dwellMs) {
  uint8_t stepSize = stepDeg ? stepDeg : 1;
  int positions[3];
  bool anyDifferent = false;

  for (int i = 0; i < 3; ++i) {
    positions[i] = servos[i].read();
    if (positions[i] != target) anyDifferent = true;
  }

  if (!anyDifferent) return;

  while (true) {
    bool movedThisPass = false;
    for (int i = 0; i < 3; ++i) {
      int diff = target - positions[i];
      if (diff == 0) continue;

      int step = abs(diff) <= stepSize ? abs(diff) : stepSize;
      positions[i] += (diff > 0) ? step : -step;
      servos[i].write(positions[i]);
      movedThisPass = true;
    }

    if (!movedThisPass) break;
    delay(dwellMs);
  }
}

void setup() {

  for (int i = 0; i < 3; ++i) {
    servos[i].attach(SERVO_PINS[i]);
  }
  writeAllServos(SERVO_OPEN_POS); // start in a welcoming, lifted leaves

  // Buttons with internal pull-ups
  for (int i = 0; i < 3; ++i) pinMode(BTN_PINS[i], INPUT_PULLUP);

  // Sonars sense when personal items are under the leaves
  for (int i = 0; i < 3; ++i) {
    pinMode(SONARS[i].trig, OUTPUT);
    digitalWrite(SONARS[i].trig, LOW);
    pinMode(SONARS[i].echo, INPUT);
  }

  // Maybe: give a grace period on power-up before the leaves are lowered
  // refractoryUntilMs = millis() + BREAK_TIME;
}


void loop() {
  // Read sonars sequentially so leaves do not hear each other’s pulses
  float d0 = readDistanceCm(SONARS[0]);
  delay(INTER_PING_DELAY_MS);
  float d1 = readDistanceCm(SONARS[1]);
  delay(INTER_PING_DELAY_MS);
  float d2 = readDistanceCm(SONARS[2]);

  uint32_t now = millis();

  // Latch each participant’s private break vote during the closed window
  if (now - lastBtnSampleMs >= BTN_SAMPLE_MS) {
    lastBtnSampleMs = now;
    for (int i = 0; i < 3; ++i) {
      if (digitalRead(BTN_PINS[i]) == LOW) votePressed[i] = true; // active LOW
    }
  }

  switch (state) {
    case OPEN: {
      // Enforce the shared break window before leaves cover items
      bool refractoryActive = ((int32_t)(now - refractoryUntilMs) < 0);
      bool allItemsDetected = allWithinThreshold(d0, d1, d2, TRIP_CM);

      if (!refractoryActive && allItemsDetected) {
        if (!closingDelayArmed) {
          closingDelayArmed = true;
          closingReadyAtMs = now + PRE_CLOSE_DELAY_MS;
        } else if ((int32_t)(now - closingReadyAtMs) >= 0) {
          moveAllServosSmooth(SERVO_CLOSED_POS, CLOSE_STEP_DEG, CLOSE_DWELL_MS);
          closingDelayArmed = false;
          closedSinceMs = now;
          for (int i = 0; i < 3; ++i) votePressed[i] = false; // new session: clear old break requests
          state = CLOSED_WAIT;
        }
      } else {
        closingDelayArmed = false;
      }
    } break;

    case CLOSED_WAIT: {
      bool timeUp   = (now - closedSinceMs >= FOCUS_TIME);
      bool votePass = allTrue(votePressed);

      if (timeUp || votePass) {
        moveAllServosSmooth(SERVO_OPEN_POS, OPEN_STEP_DEG, OPEN_DWELL_MS);
        state = OPEN;
        refractoryUntilMs = now + BREAK_TIME; // lift leaves and start shared break
        closingDelayArmed = false;
      }
    } break;
  }

  // Small idle keeps motion smooth and gives sensors breathing room
  delay(2);
}
