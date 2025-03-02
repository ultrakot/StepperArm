#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include "esp_timer.h"      // ESP timer API
#include <Preferences.h>    // For persistent storage (ESP32)

//
//------ PIN DEFINITIONS ------
// For the Xiao ESP32-C3:
#define EN_PIN       4    // Enable pin (adjust as needed)
#define DIR_PIN      2    // Direction pin
#define STEP_PIN     3    // Step pin
#define CS_PIN       20   // Chip select pin for TMC2130

// Button pins (using internal pullups)
#define BTN_PLUS   22
#define BTN_MINUS  21
#define BTN_POS1   23
#define BTN_POS2   16

//
//------ STEP & DRIVER SETTINGS ------
#define STALL_VALUE      50   // Sensitivity for StallGuard [-64..63]
#define MAX_SPEED        40   // Minimum period (faster stepping) in µs
#define MIN_SPEED      1000   // Maximum period (slower stepping) in µs
#define SG_THRESHOLD    100   // Threshold to determine stall
#define R_SENSE         0.11f // Sense resistor value

//
//------ GLOBAL VARIABLES ------

// Timer variables for continuous stepping mode:
volatile uint32_t stepPeriod = 256; // initial step period in µs
bool stepperActive = true;          // continuous stepping flag
volatile long current_position = 0;   // current position (can be negative)
volatile bool continuousDirection = true; // true: positive, false: negative

// For step counting in continuous mode:
volatile uint32_t stepCount = 0; 

// ESP timer handle:
esp_timer_handle_t step_timer;

// For button polling (store last states for edge detection)
bool lastStatePlus = HIGH, lastStateMinus = HIGH, lastStatePos1 = HIGH, lastStatePos2 = HIGH;
unsigned long pos1_press_time = 0;
unsigned long pos2_press_time = 0;
const unsigned long LONG_PRESS_TIME = 1000; // milliseconds

// For persistent storage of saved positions:
Preferences preferences;
long saved_position1 = 0;
long saved_position2 = 0;
bool saved1Valid = false;
bool saved2Valid = false;

//
//------ TMC2130 DRIVER INITIALIZATION ------
TMC2130Stepper driver(CS_PIN, R_SENSE);

//
//------ TIMER CALLBACK FOR CONTINUOUS STEPPING ------
// This function toggles the STEP_PIN. On the rising edge, it updates the position.
void onTimer(void* arg) {
  // Use a static variable to toggle state
  static bool stepState = false;
  stepState = !stepState;
  digitalWrite(STEP_PIN, stepState);
  
  if (stepState) {
    // Increment or decrement position based on current continuous direction.
    current_position += (continuousDirection ? 1 : -1);
    stepCount++;
  }
}

//
//------ BLOCKING STEP FUNCTIONS ------

// Performs a single step in the given direction (true: positive, false: negative)
void single_step(bool direction) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(5);
  current_position += (direction ? 1 : -1);
}

// Move a fixed number of steps in blocking mode.
// Before moving, we stop the continuous stepping (if active).
void move_steps(uint32_t steps, bool direction) {
  // Stop continuous stepping if running:
  if (stepperActive) {
    esp_timer_stop(step_timer);
    digitalWrite(EN_PIN, HIGH);
    stepperActive = false;
  }
  
  // Set the desired direction.
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  for (uint32_t i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(5);
    current_position += (direction ? 1 : -1);
  }
}

// Move to a saved position by calculating the steps required.
void go_to_position(long target_pos) {
  long diff = target_pos - current_position;
  bool dir = (diff > 0);
  move_steps(abs(diff), dir);
}

//
//------ STALL DETECTION & HOMING ------

// Check if a stall condition is met using StallGuard
bool check_stall() {
  TMC2130_n::DRV_STATUS_t drv_status{0};
  drv_status.sr = driver.DRV_STATUS();
  return (drv_status.sg_result < SG_THRESHOLD);
}

// Home the stepper by moving negative until stall is detected, then offset.
void home() {
  Serial.println("Starting homing sequence...");
  // Move negative until stall is detected.
  while (!check_stall()) {
    single_step(false);
    delay(1);
  }
  Serial.println("Stall detected, moving to home offset...");
  move_steps(100, true);
  current_position = 0;
  Serial.println("Homing complete");
}

//
//------ SETUP FUNCTION ------
void setup() {
  Serial.begin(250000);
  while (!Serial); // Wait for Serial Monitor
  Serial.println("\nStart...");

  // Setup stepper driver pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver (logic LOW typically enables)

  // Setup button pins with pull-up
  pinMode(BTN_PLUS, INPUT_PULLUP);
  pinMode(BTN_MINUS, INPUT_PULLUP);
  pinMode(BTN_POS1, INPUT_PULLUP);
  pinMode(BTN_POS2, INPUT_PULLUP);

  SPI.begin();

  // Initialize TMC2130 driver settings
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // mA
  driver.microsteps(16);
  driver.TCOOLTHRS(0xFFFFF); // 20-bit max value
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);

  // Create and start the ESP timer for continuous stepping
  const esp_timer_create_args_t step_timer_args = {
    .callback = &onTimer,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK, // runs in a dedicated task
    .name = "step_timer"
  };
  if (esp_timer_create(&step_timer_args, &step_timer) != ESP_OK) {
    Serial.println("Failed to create timer");
  }
  esp_timer_start_periodic(step_timer, stepPeriod);

  // Initialize persistent storage for saved positions:
  preferences.begin("stepper", false);
  saved_position1 = preferences.getLong("pos1", 0);
  saved_position2 = preferences.getLong("pos2", 0);
  saved1Valid = preferences.getBool("pos1Valid", false);
  saved2Valid = preferences.getBool("pos2Valid", false);
  Serial.print("Loaded positions: pos1=");
  Serial.print(saved1Valid ? String(saved_position1) : "none");
  Serial.print(", pos2=");
  Serial.println(saved2Valid ? String(saved_position2) : "none");
}

//
//------ MAIN LOOP ------

void loop() {
  static uint32_t lastSerialTime = 0;
  uint32_t ms = millis();

  // ---- Serial Command Processing ----
  // '0' stops continuous stepping; '1' resumes it.
  // '+' speeds up (by reducing stepPeriod); '-' slows down.
  while (Serial.available() > 0) {
    int8_t read_byte = Serial.read();
    if (read_byte == '0') {
      esp_timer_stop(step_timer);
      digitalWrite(EN_PIN, HIGH);
      stepperActive = false;
    }
    else if (read_byte == '1') {
      // Restart continuous stepping.
      digitalWrite(EN_PIN, LOW);
      esp_timer_start_periodic(step_timer, stepPeriod);
      stepperActive = true;
    }
    else if (read_byte == '+') {
      if (stepPeriod > MAX_SPEED) {
        stepPeriod -= 20;
        if (stepperActive) {
          esp_timer_stop(step_timer);
          esp_timer_start_periodic(step_timer, stepPeriod);
        }
      }
    }
    else if (read_byte == '-') {
      if (stepPeriod < MIN_SPEED) {
        stepPeriod += 20;
        if (stepperActive) {
          esp_timer_stop(step_timer);
          esp_timer_start_periodic(step_timer, stepPeriod);
        }
      }
    }
    // A simple command to initiate homing:
    else if (read_byte == 'h') {
      home();
    }
    else if (read_byte == 'm') {
      // Loop move mode: continuously move between saved pos1 and pos2.
      if (!saved1Valid) {
        Serial.println("Error: Position 1 not set.");
      } else if (!saved2Valid) {
        Serial.println("Error: Position 2 not set.");
      } else {
        Serial.println("Entering loop move mode. Send '0' to stop.");
        // In loop mode, check after each move if a stop command ('0') was received.
        while (true) {
          go_to_position(saved_position1);
          delay(100);  // short pause to allow serial input checking
          if (Serial.available() > 0) {
            char c = Serial.peek();
            if (c == '0') {  // If stop command detected, consume it and exit loop.
              Serial.read();
              Serial.println("Loop move mode stopped.");
              break;
            }
          }
          go_to_position(saved_position2);
          delay(100);
          if (Serial.available() > 0) {
            char c = Serial.peek();
            if (c == '0') {
              Serial.read();
              Serial.println("Loop move mode stopped.");
              break;
            }
          }
        }
      }
    }
    
  }

  // ---- Stall Monitoring (every 100ms) ----
  static uint32_t lastStallTime = 0;
  if ((ms - lastStallTime) > 100) {
    lastStallTime = ms;
    TMC2130_n::DRV_STATUS_t drv_status{0};
    drv_status.sr = driver.DRV_STATUS();
    Serial.print("StallGuard: ");
    Serial.print(drv_status.sg_result, DEC);
    Serial.print("   Current: ");
    Serial.println(driver.cs2rms(drv_status.cs_actual), DEC);

    // If stall is detected while continuous stepping is active, stop it.
    if (stepperActive && (drv_status.sg_result < SG_THRESHOLD)) {
      Serial.println("Stall detected! Stopping stepper.");
      esp_timer_stop(step_timer);
      digitalWrite(EN_PIN, HIGH); // disable driver
      stepperActive = false;
    }
  }

  // ---- Button Polling & Handling ----
  // Read current button states.
  bool statePlus = digitalRead(BTN_PLUS);
  bool stateMinus = digitalRead(BTN_MINUS);
  bool statePos1 = digitalRead(BTN_POS1);
  bool statePos2 = digitalRead(BTN_POS2);

  // --- BTN_PLUS: On press (transition HIGH->LOW) move +1000 steps.
  if (lastStatePlus == HIGH && statePlus == LOW) {
    Serial.println("+ step (button)");
    move_steps(1000, true);
  }
  lastStatePlus = statePlus;

  // --- BTN_MINUS: On press move -1000 steps.
  if (lastStateMinus == HIGH && stateMinus == LOW) {
    Serial.println("- step (button)");
    move_steps(1000, false);
  }
  lastStateMinus = stateMinus;

  // --- BTN_POS1: Record press time on press, then on release decide:
  if (lastStatePos1 == HIGH && statePos1 == LOW) {
    pos1_press_time = ms;
  }
  if (lastStatePos1 == LOW && statePos1 == HIGH) { // button released
    unsigned long duration = ms - pos1_press_time;
    if (duration > LONG_PRESS_TIME) {
      // Save current position as pos1.
      saved_position1 = current_position;
      saved1Valid = true;
      preferences.putLong("pos1", saved_position1);
      preferences.putBool("pos1Valid", true);
      Serial.print("Position 1 saved: ");
      Serial.println(saved_position1);
    } else if (saved1Valid) {
      // Go to saved position 1.
      Serial.println("Moving to position 1...");
      go_to_position(saved_position1);
    }
  }
  lastStatePos1 = statePos1;

  // --- BTN_POS2: Same as BTN_POS1 but for pos2.
  if (lastStatePos2 == HIGH && statePos2 == LOW) {
    pos2_press_time = ms;
  }
  if (lastStatePos2 == LOW && statePos2 == HIGH) { // button released
    unsigned long duration = ms - pos2_press_time;
    if (duration > LONG_PRESS_TIME) {
      saved_position2 = current_position;
      saved2Valid = true;
      preferences.putLong("pos2", saved_position2);
      preferences.putBool("pos2Valid", true);
      Serial.print("Position 2 saved: ");
      Serial.println(saved_position2);
    } else if (saved2Valid) {
      Serial.println("Moving to position 2...");
      go_to_position(saved_position2);
    }
  }
  lastStatePos2 = statePos2;

  // ---- Optional: Periodically report current position (every 1 sec) ----
  if (ms - lastSerialTime > 1000) {
    lastSerialTime = ms;
    Serial.print("Current Position: ");
    Serial.println(current_position);
  }
}
