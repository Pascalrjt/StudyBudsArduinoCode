#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <avr/interrupt.h>

struct LeafConfig {
  uint8_t servoPin;    // PWM pin for servo motor
  uint8_t buttonPin;   // Digital pin for break vote button (active LOW)
  uint8_t sonarTrig;   // Ultrasonic sensor trigger pin
  uint8_t sonarEcho;   // Ultrasonic sensor echo pin
};

const LeafConfig LEAVES[4] = {
  // Leaf 1: Servo, Button, Sonar(Trig, Echo)
  {13, 47, 12, 11},

  // Leaf 2: Servo, Button, Sonar(Trig, Echo)
  {10, 49, 9, 8},

  // Leaf 3: Servo, Button, Sonar(Trig, Echo)
  {7, 51, 6, 5},

  // Leaf 4: Servo, Button, Sonar(Trig, Echo)
  {4, 53, 3, 2}
};

const int pinR = 44;
const int pinG = 45;
const int pinB = 46;

// ---- LED colors (0-255) ----
// Focus (no votes): Blue #648FFF -> (100, 143, 255)
const uint8_t COLOR_FOCUS_R   = 100;
const uint8_t COLOR_FOCUS_G   = 143;
const uint8_t COLOR_FOCUS_B   = 255;

// Focus (someone voted): Yellow #FFB000 -> (255, 176, 0)
const uint8_t COLOR_VOTED_R   = 255;
const uint8_t COLOR_VOTED_G   = 176;
const uint8_t COLOR_VOTED_B   = 0;

// Break: Pink #DC267F -> (220, 38, 127)
const uint8_t COLOR_BREAK_R   = 220;
const uint8_t COLOR_BREAK_G   = 38;
const uint8_t COLOR_BREAK_B   = 127;

// Vote blink: Orange #FE6100 -> (254, 97, 0)
const uint8_t COLOR_BLINK_R   = 254;
const uint8_t COLOR_BLINK_G   = 97;
const uint8_t COLOR_BLINK_B   = 0;

// Blink timings for vote feedback
const uint16_t VOTE_BLINK_TOTAL_MS  = 1000;  // total duration of orange blink feedback
const uint16_t VOTE_BLINK_TOGGLE_MS = 250;   // on/off toggle interval during blink

// ---- Software PWM for RGB on pins 44/45/46 (Timer5 is taken by Servo) ----
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define USE_SOFTPWM_RGB 1
#else
#define USE_SOFTPWM_RGB 0
#endif

#if USE_SOFTPWM_RGB
// Pins 44,45,46 map to PORTL bits 5,4,3 respectively on Mega
#define LED_PORT   PORTL
#define LED_DDR    DDRL
#define LED_R_BIT  5  // D44
#define LED_G_BIT  4  // D45
#define LED_B_BIT  3  // D46

volatile uint8_t rgbDutyR = 0;
volatile uint8_t rgbDutyG = 0;
volatile uint8_t rgbDutyB = 0;
volatile uint8_t rgbCounter = 0; // 0..255 ramp

void initRGBSoftPWM() {
  // Ensure pins are outputs (redundant with pinMode but cheap)
  LED_DDR |= (1 << LED_R_BIT) | (1 << LED_G_BIT) | (1 << LED_B_BIT);

  // Timer2 CTC at 16 kHz: prescaler 8, OCR2A = 124 => 16e6 / (8*(124+1)) = 16 kHz
  TCCR2A = _BV(WGM21);               // CTC mode
  TCCR2B = _BV(CS21);                // prescaler 8
  OCR2A  = 124;                      // compare value
  TIMSK2 |= _BV(OCIE2A);             // enable compare match A interrupt
}

ISR(TIMER2_COMPA_vect) {
  uint8_t c = rgbCounter + 1; // 0..255
  rgbCounter = c;

  // Start of a new PWM cycle: set outputs high for non-zero duty
  if (c == 0) {
    uint8_t maskOn = 0;
    if (rgbDutyR) maskOn |= (1 << LED_R_BIT);
    if (rgbDutyG) maskOn |= (1 << LED_G_BIT);
    if (rgbDutyB) maskOn |= (1 << LED_B_BIT);
    LED_PORT |= maskOn;
  }

  // Turn channels off when the counter reaches their duty value
  uint8_t maskOff = 0;
  if (c == rgbDutyR) maskOff |= (1 << LED_R_BIT);
  if (c == rgbDutyG) maskOff |= (1 << LED_G_BIT);
  if (c == rgbDutyB) maskOff |= (1 << LED_B_BIT);
  LED_PORT &= ~maskOff;
}
#endif // USE_SOFTPWM_RGB

const uint8_t NUM_LEAVES = sizeof(LEAVES) / sizeof(LEAVES[0]); // Calculate number of leaves dynamically

const float    TRIP_CM     = 20.0;        // leaves cover items once all lie within 10 cm from ultrasonic sensor
const uint32_t FOCUS_TIME  = 240000UL;    // leaves stay closed for up to 1 min of focus
const uint32_t BREAK_TIME  = 60000UL;     // leaves stay open 10 s before next planting
const uint32_t PRE_CLOSE_DELAY_MS = 500;  // grace period from all items detected to leaves closing

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

// ---- NEW: periodic print cadence for distances ----
uint32_t lastDistancesPrintMs = 0;
const uint16_t DISTANCES_PRINT_MS = 500; // 0.2 seconds
// ---------------------------------------------------

enum LeafState { LEAF_OPEN, LEAF_CLOSED };
LeafState leafStates[NUM_LEAVES] = {LEAF_OPEN, LEAF_OPEN, LEAF_OPEN, LEAF_OPEN};

Servo servos[NUM_LEAVES];
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 columns, 2 rows

// Session management
bool sessionActive = false;
uint32_t sessionStartMs = 0;

// Vote latches (reset each time a new session starts)
bool votePressed[NUM_LEAVES] = {false, false, false, false};

// Debounce/sample cadence
uint32_t lastBtnSampleMs = 0;
const uint16_t BTN_SAMPLE_MS = 10;

uint32_t refractoryUntilMs = 0; // short shared break; ignore replanting until this time

// Per-leaf closing delay
bool closingDelayArmed[NUM_LEAVES] = {false, false, false, false};
uint32_t closingReadyAtMs[NUM_LEAVES] = {0, 0, 0, 0};

// LCD display management
uint32_t lastLCDUpdateMs = 0;
const uint16_t LCD_UPDATE_MS = 1000; // Update LCD every second
uint8_t lcdAnimationDots = 0;        // 0-3 for "Studying", "Studying.", "Studying..", "Studying..."
uint8_t breakScreenToggle = 0;       // 0-1 for alternating break screens

// LED vote-blink state
bool voteBlinkActive = false;
uint32_t voteBlinkUntilMs = 0;
uint32_t voteBlinkToggleAtMs = 0;
bool voteBlinkOn = false;

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
  //Starts the serial monitor so we print internal commands with println
  Serial.begin(9600);

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

  //Setup Pins for the changing RGB
  pinMode(pinR, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinB, OUTPUT);

#if USE_SOFTPWM_RGB
  initRGBSoftPWM();
  // Initialize LED to idle/break color on boot for immediate feedback
  setLEDColor(COLOR_BREAK_R, COLOR_BREAK_G, COLOR_BREAK_B);
#endif

  // Maybe: give a grace period on power-up before the leaves are lowered
  // refractoryUntilMs = millis() + BREAK_TIME;
}

//Sets the flower LED color given 3 parameters (R,G,B) — matches reference helper
void setLEDColor(uint8_t red, uint8_t green, uint8_t blue){
#if USE_SOFTPWM_RGB
  // Use software PWM on 44/45/46 so colors work with Servo (Timer5)
  rgbDutyR = red;
  rgbDutyG = green;
  rgbDutyB = blue;
#else
  // Fallback to hardware PWM if available
  analogWrite(pinR, red);
  analogWrite(pinG, green);
  analogWrite(pinB, blue);
#endif
}

// Decide and apply LED color based on session/break/votes, with vote-blink override
void updateLED(uint32_t now, bool refractoryActive) {
  // Handle transient orange blink on a new valid vote
  if (voteBlinkActive) {
    if ((int32_t)(now - voteBlinkUntilMs) >= 0) {
      voteBlinkActive = false; // blink window over
    } else {
      if ((int32_t)(now - voteBlinkToggleAtMs) >= 0) {
        voteBlinkToggleAtMs = now + VOTE_BLINK_TOGGLE_MS;
        voteBlinkOn = !voteBlinkOn;
      }
      if (voteBlinkOn) {
        setLEDColor(COLOR_BLINK_R, COLOR_BLINK_G, COLOR_BLINK_B);
      } else {
        setLEDColor(0, 0, 0);
      }
      return; // during blink, override baseline state
    }
  }

  // Baseline LED: Break -> Pink, Focus (voted) -> Yellow, Focus -> Blue, Idle -> Pink
  if (refractoryActive) {
    setLEDColor(COLOR_BREAK_R, COLOR_BREAK_G, COLOR_BREAK_B);
    return;
  }

  if (sessionActive) {
    bool anyValidVote = false;
    for (int i = 0; i < NUM_LEAVES; ++i) {
      if (leafStates[i] == LEAF_CLOSED && votePressed[i]) { anyValidVote = true; break; }
    }
    if (anyValidVote) {
      setLEDColor(COLOR_VOTED_R, COLOR_VOTED_G, COLOR_VOTED_B);
    } else {
      setLEDColor(COLOR_FOCUS_R, COLOR_FOCUS_G, COLOR_FOCUS_B);
    }
  } else {
    // Idle (not in break, no active session)
    setLEDColor(COLOR_BREAK_R, COLOR_BREAK_G, COLOR_BREAK_B);
  }
}

// ---- NEW: helper to print distances with pin info every 0.2s ----
void printDistances(uint32_t nowMs, const float distances[]) {
  Serial.print("Ultrasonic distances @ ");
  Serial.print(nowMs / 1000.0, 3);
  Serial.println(" s");

  for (int i = 0; i < NUM_LEAVES; ++i) {
    Serial.print("  Leaf ");
    Serial.print(i + 1);
    Serial.print(" (Trig ");
    Serial.print(LEAVES[i].sonarTrig);
    Serial.print(", Echo ");
    Serial.print(LEAVES[i].sonarEcho);
    Serial.print("): ");

    if (distances[i] >= 9998.0f) {
      Serial.println("timeout");
    } else {
      Serial.print(distances[i], 1);
      Serial.println(" cm");
    }
  }
}
// -----------------------------------------------------------------

void loop() {
  // Read sonars sequentially so leaves do not hear each other's pulses
  float distances[NUM_LEAVES];
  for (int i = 0; i < NUM_LEAVES; ++i) {
    distances[i] = readDistanceCm(LEAVES[i]);
    if (i < NUM_LEAVES - 1) delay(INTER_PING_DELAY_MS);
  }

  uint32_t now = millis();

  // ---- NEW: periodic serial print of distances and pins (every 0.2 s) ----
  if (now - lastDistancesPrintMs >= DISTANCES_PRINT_MS) {
    lastDistancesPrintMs = now;
    printDistances(now, distances);
  }
  // ------------------------------------------------------------------------

  // Update LCD display
  updateLCDDisplay(now);

  // Latch each participant's private break vote during an active session
  if (now - lastBtnSampleMs >= BTN_SAMPLE_MS) {
    lastBtnSampleMs = now;
    if (sessionActive) {
      for (int i = 0; i < NUM_LEAVES; ++i) {
        bool pressed = (digitalRead(LEAVES[i].buttonPin) == LOW); // active LOW
        // A vote is valid only if the corresponding leaf is currently closed
        if (pressed && !votePressed[i] && leafStates[i] == LEAF_CLOSED) {
          votePressed[i] = true;
          // Start transient orange blink to acknowledge the new valid vote
          voteBlinkActive = true;
          voteBlinkUntilMs = now + VOTE_BLINK_TOTAL_MS;
          voteBlinkToggleAtMs = now; // toggle immediately on next update
          voteBlinkOn = true;
        }
      }
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
      // End any pending vote blink when session transitions to break
      voteBlinkActive = false;
    }
  }

  // Recompute refractory flag after any possible changes above to ensure immediate LED update
  bool refractoryForLED = ((int32_t)(now - refractoryUntilMs) < 0);
  // Apply LED color for the current state (runs after state transitions above)
  updateLED(now, refractoryForLED);

  // Small idle keeps motion smooth and gives sensors breathing room
  delay(2);
}
