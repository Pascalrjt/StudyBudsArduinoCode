#include <Servo.h>
#include <LiquidCrystal_I2C.h>

struct LeafConfig {
  uint8_t servoPin;    // PWM pin for servo motor
  uint8_t buttonPin;   // Digital pin for break vote button (active LOW)
  uint8_t sonarTrig;   // Ultrasonic sensor trigger pin
  uint8_t sonarEcho;   // Ultrasonic sensor echo pin
};

const LeafConfig LEAVES[3] = {
  // Leaf 1: Servo, Button, Sonar(Trig, Echo)
  {3, 13, 6, 5},

  // Leaf 2: Servo, Button, Sonar(Trig, Echo)
  {2, 11, 8, 7},

  // Leaf 3: Servo, Button, Sonar(Trig, Echo)
  {4, 12, 10, 9}
};

const uint8_t NUM_LEAVES = sizeof(LEAVES) / sizeof(LEAVES[0]); // Calculate number of leaves dynamically

const float    TRIP_CM     = 10.0;        // leaves cover items once all lie within 10 cm from ultrasonic sensor
const uint32_t FOCUS_TIME  = 60000UL;     // leaves stay closed for up to 1 min of focus
const uint32_t BREAK_TIME  = 10000UL;     // leaves stay open 10 s before next planting
const uint32_t PRE_CLOSE_DELAY_MS = 500; // grace period from all items detected to leaves closing

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

enum LeafState { LEAF_OPEN, LEAF_CLOSED };
LeafState leafStates[NUM_LEAVES] = {LEAF_OPEN, LEAF_OPEN, LEAF_OPEN};

Servo servos[NUM_LEAVES];
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 columns, 2 rows

// Session management
bool sessionActive = false;
uint32_t sessionStartMs = 0;

// Vote latches (reset each time a new session starts)
bool votePressed[NUM_LEAVES] = {false, false, false};

// Debounce/sample cadence
uint32_t lastBtnSampleMs = 0;
const uint16_t BTN_SAMPLE_MS = 10;

uint32_t refractoryUntilMs = 0; // short shared break; ignore replanting until this time

// Per-leaf closing delay
bool closingDelayArmed[NUM_LEAVES] = {false, false, false};
uint32_t closingReadyAtMs[NUM_LEAVES] = {0, 0, 0};

// LCD display management
uint32_t lastLCDUpdateMs = 0;
const uint16_t LCD_UPDATE_MS = 1000; // Update LCD every second
uint8_t lcdAnimationDots = 0;        // 0-3 for "Studying", "Studying.", "Studying..", "Studying..."
uint8_t breakScreenToggle = 0;       // 0-1 for alternating break screens

float readDistanceCm(const LeafConfig& leaf) {
  // Trigger pulse
  pinMode(leaf.sonarTrig, OUTPUT);
  digitalWrite(leaf.sonarTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(leaf.sonarTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(leaf.sonarTrig, LOW);

  // Listen for echo
  pinMode(leaf.sonarEcho, INPUT);
  unsigned long duration = pulseIn(leaf.sonarEcho, HIGH, PULSE_TIMEOUT_US);
  if (duration == 0) return 9999.0f; // timeout -> no item planted over this leaf
  return duration / 58.0f;           // ~58 us per cm (round trip)
}

inline bool allTrue(const bool arr[3]) {
  return arr[0] && arr[1] && arr[2];
}

bool allClosedLeavesVoted() {
  // Check if all closed leaves have corresponding votes
  for (int i = 0; i < NUM_LEAVES; ++i) {
    if (leafStates[i] == LEAF_CLOSED && !votePressed[i]) {
      return false; // this closed leaf hasn't voted yet
    }
  }
  return true; // all closed leaves have voted (or no leaves are closed)
}

inline void writeAllServos(int pos) {
  for (int i = 0; i < NUM_LEAVES; ++i) servos[i].write(pos);
}

void moveServoSmooth(int leafIndex, int target, uint8_t stepDeg, uint16_t dwellMs) {
  uint8_t stepSize = stepDeg ? stepDeg : 1;
  int currentPos = servos[leafIndex].read();

  if (currentPos == target) return;

  while (currentPos != target) {
    int diff = target - currentPos;
    int step = abs(diff) <= stepSize ? abs(diff) : stepSize;
    currentPos += (diff > 0) ? step : -step;
    servos[leafIndex].write(currentPos);
    delay(dwellMs);
  }
}

void moveAllServosSmooth(int target, uint8_t stepDeg, uint16_t dwellMs) {
  uint8_t stepSize = stepDeg ? stepDeg : 1;
  int positions[NUM_LEAVES];
  bool anyDifferent = false;

  for (int i = 0; i < NUM_LEAVES; ++i) {
    positions[i] = servos[i].read();
    if (positions[i] != target) anyDifferent = true;
  }

  if (!anyDifferent) return;

  while (true) {
    bool movedThisPass = false;
    for (int i = 0; i < NUM_LEAVES; ++i) {
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

// LCD Display Functions
void updateLCDDisplay(uint32_t now) {
  if (now - lastLCDUpdateMs < LCD_UPDATE_MS) return;
  lastLCDUpdateMs = now;

  lcd.clear();

  if (sessionActive) {
    // Study mode: show studying animation and leaf count
    displayStudyMode();
  } else {
    // Break/Idle mode: alternate between welcome message and timer
    displayBreakMode(now);
  }
}

void displayStudyMode() {
  // Count closed leaves
  uint8_t closedCount = 0;
  for (int i = 0; i < NUM_LEAVES; ++i) {
    if (leafStates[i] == LEAF_CLOSED) closedCount++;
  }

  // Top row: "Studying..." with animated dots
  lcd.setCursor(0, 0);
  lcd.print("Studying");
  for (uint8_t i = 0; i < lcdAnimationDots; ++i) {
    lcd.print(".");
  }

  // Bottom row: "X/NUM_LEAVES leaves used"
  lcd.setCursor(0, 1);
  lcd.print(closedCount);
  lcd.print("/");
  lcd.print(NUM_LEAVES);
  lcd.print(" leaves used");

  // Cycle animation dots (0->1->2->3->0)
  lcdAnimationDots = (lcdAnimationDots + 1) % 4;
}

void displayBreakMode(uint32_t now) {
  if (breakScreenToggle == 0) {
    // Screen 1: Welcome message
    lcd.setCursor(0, 0);
    lcd.print("Place items in");
    lcd.setCursor(0, 1);
    lcd.print("leaves to start!");
  } else {
    // Screen 2: Show break time remaining
    lcd.setCursor(0, 0);
    lcd.print("Break time!");
    lcd.setCursor(0, 1);

    // Calculate remaining break time
    if ((int32_t)(now - refractoryUntilMs) < 0) {
      uint32_t remainingMs = refractoryUntilMs - now;
      uint8_t remainingSec = (remainingMs + 999) / 1000; // Round up
      lcd.print("Ready in ");
      lcd.print(remainingSec);
      lcd.print("s");
    } else {
      lcd.print("Ready to start!");
    }
  }

  // Toggle between screens every update
  breakScreenToggle = (breakScreenToggle + 1) % 2;
}

void setup() {
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("StudyBuds V2");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  for (int i = 0; i < NUM_LEAVES; ++i) {
    servos[i].attach(LEAVES[i].servoPin);
  }
  writeAllServos(SERVO_OPEN_POS); // start in a welcoming, lifted leaves

  // Buttons with internal pull-ups
  for (int i = 0; i < NUM_LEAVES; ++i) pinMode(LEAVES[i].buttonPin, INPUT_PULLUP);

  // Sonars sense when personal items are under the leaves
  for (int i = 0; i < NUM_LEAVES; ++i) {
    pinMode(LEAVES[i].sonarTrig, OUTPUT);
    digitalWrite(LEAVES[i].sonarTrig, LOW);
    pinMode(LEAVES[i].sonarEcho, INPUT);
  }

  delay(1000); // Show welcome message briefly
  lcd.clear();

  // Maybe: give a grace period on power-up before the leaves are lowered
  // refractoryUntilMs = millis() + BREAK_TIME;
}


void loop() {
  // Read sonars sequentially so leaves do not hear each other's pulses
  float distances[NUM_LEAVES];
  for (int i = 0; i < NUM_LEAVES; ++i) {
    distances[i] = readDistanceCm(LEAVES[i]);
    if (i < NUM_LEAVES - 1) delay(INTER_PING_DELAY_MS);
  }

  uint32_t now = millis();

  // Update LCD display
  updateLCDDisplay(now);

  // Latch each participant's private break vote during an active session
  if (now - lastBtnSampleMs >= BTN_SAMPLE_MS) {
    lastBtnSampleMs = now;
    for (int i = 0; i < NUM_LEAVES; ++i) {
      if (digitalRead(LEAVES[i].buttonPin) == LOW) votePressed[i] = true; // active LOW
    }
  }

  // Check if we're in refractory period (shared break time)
  bool refractoryActive = ((int32_t)(now - refractoryUntilMs) < 0);

  // Handle individual leaf closing when not in refractory period
  if (!refractoryActive) {
    for (int i = 0; i < NUM_LEAVES; ++i) {
      if (leafStates[i] == LEAF_OPEN && distances[i] <= TRIP_CM) {
        // Item detected for this leaf
        if (!closingDelayArmed[i]) {
          closingDelayArmed[i] = true;
          closingReadyAtMs[i] = now + PRE_CLOSE_DELAY_MS;
        } else if ((int32_t)(now - closingReadyAtMs[i]) >= 0) {
          // Grace period elapsed, close this leaf
          moveServoSmooth(i, SERVO_CLOSED_POS, CLOSE_STEP_DEG, CLOSE_DWELL_MS);
          leafStates[i] = LEAF_CLOSED;
          closingDelayArmed[i] = false;

          // Start session if this is the first leaf to close
          if (!sessionActive) {
            sessionActive = true;
            sessionStartMs = now;
            for (int j = 0; j < NUM_LEAVES; ++j) votePressed[j] = false; // clear votes for new session
          }
        }
      } else if (leafStates[i] == LEAF_OPEN) {
        // Item moved away, cancel closing delay for this leaf
        closingDelayArmed[i] = false;
      }
    }
  }

  // Handle session end (all leaves open together)
  if (sessionActive) {
    bool timeUp = (now - sessionStartMs >= FOCUS_TIME);
    bool votePass = allClosedLeavesVoted();

    if (timeUp || votePass) {
      // Open all leaves together
      moveAllServosSmooth(SERVO_OPEN_POS, OPEN_STEP_DEG, OPEN_DWELL_MS);

      // Reset all leaf states
      for (int i = 0; i < NUM_LEAVES; ++i) {
        leafStates[i] = LEAF_OPEN;
        closingDelayArmed[i] = false;
      }

      sessionActive = false;
      refractoryUntilMs = now + BREAK_TIME; // start shared break period
    }
  }

  // Small idle keeps motion smooth and gives sensors breathing room
  delay(2);
}
