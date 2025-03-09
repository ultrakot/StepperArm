#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <Preferences.h>
#include <Wire.h>
#include "Adafruit_MPR121.h"

// Include ESP-IDF header for GPIO interrupt services:
#include "driver/gpio.h"

// Pin definitions for Xiao ESP32-C3:
#define EN_PIN         4    // Enable pin
#define DIR_PIN        2    // Direction pin
#define STEP_PIN       3    // Step pin
#define CS_PIN         20   // Chip select pin for TMC2130

// Button pin definitions (using GPIO numbers)
#define BUTTON_PIN_4   7    // Command: move -5000 steps (same as serial '4')
#define BUTTON_PIN_3   21   // Command: move +5000 steps (same as serial '3')
#define BUTTON_POS_PIN 6    // Saves position: alternates between pos1 and pos2
#define BUTTON_LOOP_PIN 5   // Triggers loop mode

// Driver settings:
#define STALL_VALUE    0
#define R_SENSE        0.11f
#define SG_THRESHOLD   50  // StallGuard threshold

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

// ----------------------------
// Button Interrupt Variables
// ----------------------------
// Flags set by the ISRs.
volatile bool button4Flag = false;
volatile bool button3Flag = false;
volatile bool buttonPosFlag = false;
volatile bool buttonLoopFlag = false;

// For BUTTON_POS cycle: 0 = next press saves pos1, 1 = next press saves pos2.
volatile int posCycle = 0;

// For basic debouncing inside the ISR.
volatile unsigned long lastInterruptTime4   = 0;
volatile unsigned long lastInterruptTime3   = 0;
volatile unsigned long lastInterruptTimePos   = 0;
volatile unsigned long lastInterruptTimeLoop  = 0;
#define DEBOUNCE_TIME 50000UL  // 50 ms in microseconds

// ----------------------------
// ISRs for the buttons using ESP32-C3 native routines
// ----------------------------
void IRAM_ATTR isrButton4(void* arg) {
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime4 > DEBOUNCE_TIME) {
    button4Flag = true;
    lastInterruptTime4 = currentTime;
  }
}

void IRAM_ATTR isrButton3(void* arg) {
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime3 > DEBOUNCE_TIME) {
    button3Flag = true;
    lastInterruptTime3 = currentTime;
  }
}

void IRAM_ATTR isrButtonPos(void* arg) {
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTimePos > DEBOUNCE_TIME) {
    buttonPosFlag = true;
    lastInterruptTimePos = currentTime;
  }
}

void IRAM_ATTR isrButtonLoop(void* arg) {
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTimeLoop > DEBOUNCE_TIME) {
    buttonLoopFlag = true;
    lastInterruptTimeLoop = currentTime;
  }
}

// ----------------------------
// Movement and Utility Functions
// ----------------------------

// Blocking step movement: moves a given number of steps.
void move_steps(long steps, bool direction, int microtime) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(microtime);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(microtime);
    current_position += (direction ? -1 : 1);
    if (i > 0 && i % 15 == 0 && microtime > 160) {
      microtime--;
    }
  }
}

// Moves to a target position by calculating the required steps.
void go_to_position(long target) {
  digitalWrite(EN_PIN, LOW); // Enable stepper
  long diff = target - current_position;
  bool direction = (diff < 0);
  move_steps(abs(diff), direction, 160);
  digitalWrite(EN_PIN, HIGH); // Disable stepper
}

// Homing routine.
void home() {
  Serial.println("Homing...");
  digitalWrite(EN_PIN, LOW); // Enable stepper

  int microtime = 300; 
  digitalWrite(DIR_PIN, HIGH);
  for (long i = 0; i < 50; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(160);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(160);
  }
  while (true) {
    TMC2130_n::DRV_STATUS_t status{0};
    status.sr = driver.DRV_STATUS();
    Serial.print("values : ");
    Serial.print(status.sg_result, DEC);
    Serial.print(" cs_actual :");
    Serial.println(driver.cs2rms(status.cs_actual), DEC);
    if (microtime > 160) microtime = microtime - 1;
    Serial.println("current position");
    Serial.println(current_position);
    for (long i = 0; i < 15; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(160);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(160);
      current_position--;
    }
    Serial.println("current position");
    Serial.println(current_position);
    if (status.sg_result < SG_THRESHOLD || Serial.read() == '0') break;
  }
  current_position = 0;
  Serial.print("Current position: ");
  Serial.println(current_position);
  digitalWrite(EN_PIN, HIGH);  // Disable stepper 
  Serial.println("Homing complete.");
}

// Loop mode function: continuously moves between pos1 and pos2 with a given delay.
void loopModeFunction(unsigned long delayTime) {
  if (!pos1Valid || !pos2Valid) {
    Serial.println("Both positions must be set before moving.");
    return;
  }
  Serial.println("Loop mode started.");
  digitalWrite(EN_PIN, LOW); // Enable stepper
  while (true) {
    go_to_position(pos1);
    unsigned long start = millis();
    while (millis() - start < delayTime) {
      if (buttonLoopFlag) {  // Use the loop button flag to exit loop mode
        buttonLoopFlag = false;
        Serial.println("Loop mode stop requested.");
        digitalWrite(EN_PIN, HIGH);
        return;
      }
      delay(10);
    }
    go_to_position(pos2);
    start = millis();
    while (millis() - start < delayTime) {
      if (buttonLoopFlag) {
        buttonLoopFlag = false;
        Serial.println("Loop mode stop requested.");
        digitalWrite(EN_PIN, HIGH);
        return;
      }
      delay(10);
    }
  }
}

// ----------------------------
// Command Handler Functions
// ----------------------------

// Save current position as pos1.
void handleSavePos1() {
  pos1 = current_position;
  pos1Valid = true;
  preferences.putLong("pos1", pos1);
  preferences.putBool("pos1Valid", true);
  Serial.print("Position 1 saved: ");
  Serial.println(pos1);
}

// Save current position as pos2.
void handleSavePos2() {
  pos2 = current_position;
  pos2Valid = true;
  preferences.putLong("pos2", pos2);
  preferences.putBool("pos2Valid", true);
  Serial.print("Position 2 saved: ");
  Serial.println(pos2);
}

// Move +5000 steps.
void handleMovePlus5000() {
  digitalWrite(EN_PIN, LOW);
  Serial.println("Moving +5000 steps...");
  move_steps(5000, true, 300);
  Serial.print("Current position: ");
  Serial.println(current_position);
  digitalWrite(EN_PIN, HIGH);
}

// Move -5000 steps.
void handleMoveMinus5000() {
  digitalWrite(EN_PIN, LOW);
  Serial.println("Moving -5000 steps...");
  move_steps(5000, false, 300);
  Serial.print("Current position: ");
  Serial.println(current_position);
  digitalWrite(EN_PIN, HIGH);
}

// Home routine.
void handleHome() {
  home();
}

// Enter loop mode.
void handleLoopMode() {
  loopModeFunction(2000); // Delay between moves: 2000ms.
}

// Stop command.
void handleStop() {
  Serial.println("Stop command received.");
  digitalWrite(EN_PIN, HIGH);
}

// Start command.
void handleStart() {
  Serial.println("Start command received.");
  digitalWrite(EN_PIN, LOW);
}

// ----------------------------
// Setup and Main Loop
// ----------------------------
void setup() {
  Serial.begin(250000);
  while (!Serial);
  Serial.println("Start...");

  // Set up stepper control pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);  // Disable driver initially

  // Set up button pins with internal pull-ups
  pinMode(BUTTON_PIN_4, INPUT_PULLUP);
  pinMode(BUTTON_PIN_3, INPUT_PULLUP);
  pinMode(BUTTON_POS_PIN, INPUT_PULLUP);
  pinMode(BUTTON_LOOP_PIN, INPUT_PULLUP);

  // Install the ESP32-C3 GPIO ISR service.
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

  // Add handlers for each button using ESP-IDF style:
  gpio_isr_handler_add((gpio_num_t)BUTTON_PIN_4, isrButton4, NULL);
  gpio_isr_handler_add((gpio_num_t)BUTTON_PIN_3, isrButton3, NULL);
  gpio_isr_handler_add((gpio_num_t)BUTTON_POS_PIN, isrButtonPos, NULL);
  gpio_isr_handler_add((gpio_num_t)BUTTON_LOOP_PIN, isrButtonLoop, NULL);

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

  //homing
  home();
}

void loop() {
  // Process serial commands.
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1':  handleSavePos1();      break;
      case '2':  handleSavePos2();      break;
      case '3':  handleMovePlus5000();  break;
      case '4':  handleMoveMinus5000(); break;
      case 'h':  handleHome();          break;
      case 'm':  handleLoopMode();      break;
      case '0':  handleStop();          break;
      case '5':  handleStart();         break;
      default:
        Serial.print("Unknown command: ");
        Serial.println(cmd);
        break;
    }
  }

  // Process button interrupt flags.
  if (button4Flag) {
    button4Flag = false;
    handleMoveMinus5000();
  }
  if (button3Flag) {
    button3Flag = false;
    handleMovePlus5000();
  }
  if (buttonPosFlag) {
    // Alternate saving positions on GPIO6.
    if (posCycle == 0) {
      handleSavePos1();
      posCycle = 1;
    } else {
      handleSavePos2();
      posCycle = 0;
    }
    buttonPosFlag = false;
  }
  if (buttonLoopFlag) {
    buttonLoopFlag = false;
    handleLoopMode();
  }
}
