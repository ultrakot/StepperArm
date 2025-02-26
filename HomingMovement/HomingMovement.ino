#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include "esp_timer.h" // ESP timer API

// Adjust pins for the Xiao ESP32-C3
#define EN_PIN       4    // Enable pin (adjust as needed)
#define DIR_PIN      2     // Direction pin (adjust as needed)
#define STEP_PIN     3     // Step pin (adjust as needed)
#define CS_PIN       20     // Chip select pin

#define STALL_VALUE      50  // Sensitivity for StallGuard [-64..63]
#define MAX_SPEED        40   // Minimum period (faster stepping) in µs
#define MIN_SPEED      1000   // Maximum period (slower stepping) in µs

#define R_SENSE 0.11f  // Sense resistor value

// Set a StallGuard threshold below which we consider the motor stalled.
// You may need to adjust this value based on your motor, load, and settings.
#define SG_THRESHOLD 100  

// Initialize the TMC2130 driver on hardware SPI
TMC2130Stepper driver(CS_PIN, R_SENSE);

// Global variables
volatile uint32_t stepPeriod = 256; // initial step period in microseconds
bool stepperActive = true;          // flag indicating if the stepper is running

// Handle for the ESP timer
esp_timer_handle_t step_timer;

// Timer callback: toggle the STEP_PIN to generate step pulses
void onTimer(void* arg) {
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
}

void setup() {
  Serial.begin(250000);
  while (!Serial); // Wait for Serial Monitor
  Serial.println("\nStart...");

  // Setup pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver (logic LOW typically enables)

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

  // Create and start the ESP timer to generate step pulses
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
}

void loop() {
  static uint32_t last_time = 0;
  uint32_t ms = millis();

  // Process serial commands to start/stop or change speed.
  // '0' stops the stepper, '1' resumes, '+' speeds up, '-' slows down.
  while (Serial.available() > 0) {
    int8_t read_byte = Serial.read();
    #ifdef USING_TMC2660
      if (read_byte == '0') {
        esp_timer_stop(step_timer);
        driver.toff(0);
        stepperActive = false;
      }
      else if (read_byte == '1') {
        esp_timer_start_periodic(step_timer, stepPeriod);
        driver.toff(driver.savedToff());
        stepperActive = true;
      }
    #else
      if (read_byte == '0') {
        esp_timer_stop(step_timer);
        digitalWrite(EN_PIN, HIGH);
        stepperActive = false;
      }
      else if (read_byte == '1') {
        esp_timer_start_periodic(step_timer, stepPeriod);
        digitalWrite(EN_PIN, LOW);
        stepperActive = true;
      }
    #endif
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
  }

  // Every 100ms, read the StallGuard value and current,
  // then check if the stall condition is met.
  if ((ms - last_time) > 100) {
    last_time = ms;
    TMC2130_n::DRV_STATUS_t drv_status{0};
    drv_status.sr = driver.DRV_STATUS();

    Serial.print("0 ");
    Serial.print(drv_status.sg_result, DEC);
    Serial.print(" ");
    Serial.println(driver.cs2rms(drv_status.cs_actual), DEC);

    // If the stall condition is detected, stop the stepper.
    if (stepperActive && drv_status.sg_result < SG_THRESHOLD) {
      Serial.println("Stall detected! Stopping stepper.");
      esp_timer_stop(step_timer);
      digitalWrite(EN_PIN, HIGH); // disable driver
      stepperActive = false;
    }
  }
}