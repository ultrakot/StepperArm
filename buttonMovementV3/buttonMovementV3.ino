#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <Preferences.h>

// Pin definitions for Xiao ESP32-C3:
#define EN_PIN    4    // Enable pin
#define DIR_PIN   2    // Direction pin
#define STEP_PIN  3    // Step pin
#define CS_PIN    20   // Chip select pin for TMC2130

// Driver settings:
#define STALL_VALUE 0
#define R_SENSE     0.11f
#define SG_THRESHOLD 100  // StallGuard threshold

// Global position tracking
long current_position = 0;

// Saved positions and flags
long pos1 = 0;
long pos2 = 0;
bool pos1Valid = false;
bool pos2Valid = false;

// TMC2130 driver instance
TMC2130Stepper driver(CS_PIN, R_SENSE);

// For persistent storage:
Preferences preferences;

// Blocking step movement: moves a given number of steps.
// 'direction' is true for forward (+) and false for reverse (–).
void move_steps(long steps, bool direction) {
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(250 - i);  // Minimal HIGH pulse
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(250 - i);  // Minimal LOW pulse
    current_position += (direction ? 1 : -1);
  }
  digitalWrite(EN_PIN, HIGH);
}

// Moves to a target position by calculating the required steps.
void go_to_position(long target) {
  long diff = target - current_position;
  bool direction = (diff >= 0);
  move_steps(abs(diff), direction);
}

// Simple homing: moves in the negative direction until a stall is detected,
// then moves forward a fixed offset and resets position to zero.
void home() {
  Serial.println("Homing...");
  digitalWrite(EN_PIN, LOW);

  while (true) {
    TMC2130_n::DRV_STATUS_t status{0};
    status.sr = driver.DRV_STATUS();
    Serial.print("values : ");
    Serial.print(status.sg_result, DEC);
    Serial.print(" cs_actual :");
    Serial.println(driver.cs2rms(status.cs_actual), DEC);

    
    // Move one step in the negative direction:
    move_steps(15, true);
    // digitalWrite(DIR_PIN, HIGH);
    // digitalWrite(STEP_PIN, HIGH);
    // delayMicroseconds(200);
    // digitalWrite(STEP_PIN, LOW);
    // delayMicroseconds(200);
    current_position--;  // Update position

    if (status.sg_result < SG_THRESHOLD) break;  // Stall detected
  }
  // Offset from stall point:
  Serial.println("Ofset from stall.");
  //move_steps(5000, true);
  //current_position = 0;
  Serial.print("Current position: ");
  Serial.println(current_position);
  digitalWrite(EN_PIN, HIGH);
  Serial.println("Homing complete.");
}

void setup() {
  Serial.begin(250000);
  while (!Serial);
  Serial.println("Start...");

  // Set up pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);  // Enable the driver

  SPI.begin();
  
  // Initialize TMC2130 driver settings
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(1500); // in mA
  driver.microsteps(4);
  driver.TCOOLTHRS(0xFFFFF);
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);

  // Load saved positions from nonvolatile storage
  preferences.begin("stepper", false);
  pos1 = preferences.getLong("pos1", 0);
  pos2 = preferences.getLong("pos2", 0);
  pos1Valid = preferences.getBool("pos1Valid", false);
  pos2Valid = preferences.getBool("pos2Valid", false);
  Serial.print("Loaded positions: pos1=");
  Serial.print(pos1Valid ? String(pos1) : "none");
  Serial.print(", pos2=");
  Serial.println(pos2Valid ? String(pos2) : "none");
}

void loop() {
  // Process serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1': {
        // Save current position as pos1
        pos1 = current_position;
        pos1Valid = true;
        preferences.putLong("pos1", pos1);
        preferences.putBool("pos1Valid", true);
        Serial.print("Position 1 saved: ");
        Serial.println(pos1);
        break;
      }
      case '2': {
        // Save current position as pos2
        pos2 = current_position;
        pos2Valid = true;
        preferences.putLong("pos2", pos2);
        preferences.putBool("pos2Valid", true);
        Serial.print("Position 2 saved: ");
        Serial.println(pos2);
        break;
      }
      case '3': {
        // Move +1000 steps
        Serial.println("Moving +1000 steps...");
        move_steps(150, true);
        Serial.print("Current position: ");
        Serial.println(current_position);
        break;
      }
      case '4': {
        // Move -100 steps
        Serial.println("Moving -100 steps...");
        move_steps(150, false);
        Serial.print("Current position: ");
        Serial.println(current_position);
        break;
      }
      case 'h': {
        home();
        break;
      }
      case 'm': {
        // Loop mode: continuously move between pos1 and pos2.
        if (!pos1Valid || !pos2Valid) {
          Serial.println("Both positions must be set before moving.");
        } else {
          Serial.println("Entering loop move mode. Send '0' to stop.");
          while (true) {
            go_to_position(pos1);
            if (Serial.available() > 0 && Serial.peek() == '0') {
              Serial.read();
              Serial.println("Loop move mode stopped.");
              break;
            }
            go_to_position(pos2);
            if (Serial.available() > 0 && Serial.peek() == '0') {
              Serial.read();
              Serial.println("Loop move mode stopped.");
              break;
            }
          }
        }
        break;
      }
      case '0': {
        Serial.println("Stop command received.");
        digitalWrite(EN_PIN, HIGH);
        // In this simplified version the '0' command stops the loop mode (if active).
        break;
      }
      case '5': {
        Serial.println("start command received.");
        digitalWrite(EN_PIN, LOW);
        // In this simplified version the '0' command stops the loop mode (if active).
        break;
      }
      default:
        Serial.print("Unknown command: ");
        Serial.println(cmd);
        break;
    }
  }
}
