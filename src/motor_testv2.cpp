/*

#include <Arduino.h>
#include <Servo.h>
#include <FreeRTOS.h>
#include <task.h>
#include <Wire.h>
#include <AS5600.h>

#include "pins.h"


// ---------------------------------------------
// GLOBAL VARIABLES
// ---------------------------------------------
// Stepper parameters
int baseStepDelay = 500;  
int armStepDelay  = 500;

bool baseMoving = false;
bool armMoving  = false;

int baseDirection = LOW;
int armDirection  = LOW;

// Servos
Servo baseA;
Servo baseB;
Servo middleS;
Servo wristS;
Servo clawS;

// AS5600 mag encoder
AS5600 encoder;

// ---------------------------------------------
// FREERTOS TASK DECLARATIONS
// ---------------------------------------------
void TaskSerialCommands(void *pvParameters);
void TaskStepperControl(void *pvParameters);

// ---------------------------------------------
// SETUP
// ---------------------------------------------
void setup() {
  // Base stepper pins
  pinMode(BASE_STEP_PIN, OUTPUT);
  pinMode(BASE_DIR_PIN, OUTPUT);
  pinMode(BASE_EN_PIN, OUTPUT);

  // Arm stepper pins
  pinMode(ARM_STEP_PIN, OUTPUT);
  pinMode(ARM_DIR_PIN, OUTPUT);
  pinMode(ARM_EN_PIN, OUTPUT);

  // Setup AS5600 encoder
  Wire.begin();
  while (1) {
  if (!encoder.begin()) {
    Serial.println("AS5600 not detected!");
  } else {
    break;
  }
}

  digitalWrite(BASE_EN_PIN, LOW); // enable drivers
  digitalWrite(ARM_EN_PIN, LOW);

  // Attach servos
  baseA.attach(BASE_SERVO_A);
  baseB.attach(BASE_SERVO_B);
  middleS.attach(MIDDLE_SERVO);
  wristS.attach(WRIST_SERVO);
  clawS.attach(CLAW_SERVO);

  // Default servo positions
  baseA.write(90);
  baseB.write(90);
  middleS.write(90);
  wristS.write(90);
  clawS.write(90);

  Serial.begin(115200);
  Serial.println("Robotic Arm Controller Ready");

  Serial.println("\nSTEPPER COMMANDS:");
  Serial.println("  F = Base forward");
  Serial.println("  B = Base backward");
  Serial.println("  G = Base stop");
  Serial.println("  f = Arm forward");
  Serial.println("  b = Arm backward");
  Serial.println("  g = Arm stop");
  Serial.println("  + / - = Base speed");
  Serial.println("  ] / [ = Arm speed");

  Serial.println("\nSERVO COMMANDS:");
  Serial.println("  Axxx = Base servo angle (xxx)");
  Serial.println("  Mxxx = Middle servo angle");
  Serial.println("  Wxxx = Wrist servo angle");
  Serial.println("  Cxxx = Claw servo angle");

  // Create FreeRTOS tasks
  xTaskCreate(
    TaskSerialCommands,
    "SerialCmd",
    256,  // Stack size
    NULL,
    2,    // Priority (higher)
    NULL
  );

  xTaskCreate(
    TaskStepperControl,
    "StepperCtrl",
    128,  // Stack size
    NULL,
    1,    // Priority (lower)
    NULL
  );

  // Start the scheduler
  vTaskStartScheduler();
}

// ---------------------------------------------
// STEPPER HELPERS
// ---------------------------------------------
void stepBase() {
  digitalWrite(BASE_STEP_PIN, HIGH);
  delayMicroseconds(baseStepDelay);
  digitalWrite(BASE_STEP_PIN, LOW);
  delayMicroseconds(baseStepDelay);
}

void stepArm() {
  digitalWrite(ARM_STEP_PIN, HIGH);
  delayMicroseconds(armStepDelay);
  digitalWrite(ARM_STEP_PIN, LOW);
  delayMicroseconds(armStepDelay);
}

void moveServoSmooth(Servo &s, int target, int stepDelayMs = 10) {
  int current = s.read();
  if (current == target) return;

  if (current < target) {
    for (int pos = current; pos <= target; pos++) {
      s.write(pos);
      delay(stepDelayMs);
    }
  } else {
    for (int pos = current; pos >= target; pos--) {
      s.write(pos);
      delay(stepDelayMs);
    }
  }
}

// ---------------------------------------------
// MAIN LOOP
// ---------------------------------------------
void loop() {
  // Empty - FreeRTOS tasks handle everything
}

// ---------------------------------------------
// FREERTOS TASK: Serial Command Processing
// ---------------------------------------------
void TaskSerialCommands(void *pvParameters) {
  (void) pvParameters;

  while (1) {
    // Process serial commands
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');

      // ----------------------
      // STEPPER COMMANDS
      // ----------------------

      if (cmd == "F") {  // BASE forward
        baseMoving = true;
        baseDirection = LOW;
        digitalWrite(BASE_DIR_PIN, baseDirection);
        Serial.println("Base moving forward");
      }
      else if (cmd == "B") {  // BASE backward
        baseMoving = true;
        baseDirection = HIGH;
        digitalWrite(BASE_DIR_PIN, baseDirection);
        Serial.println("Base moving backward");
      }
      else if (cmd == "G") {  // BASE stop
        baseMoving = false;
        Serial.println("Base stopped");
      }

      else if (cmd == "f") {  // ARM forward
        armMoving = true;
        armDirection = LOW;
        digitalWrite(ARM_DIR_PIN, armDirection);
        Serial.println("Arm moving forward");
      }
      else if (cmd == "b") {  // ARM backward
        armMoving = true;
        armDirection = HIGH;
        digitalWrite(ARM_DIR_PIN, armDirection);
        Serial.println("Arm moving backward");
      }
      else if (cmd == "g") {  // ARM stop
        armMoving = false;
        Serial.println("Arm stopped");
      }

      // Speed control
      else if (cmd == "+") baseStepDelay = max(100, baseStepDelay - 50);
      else if (cmd == "-") baseStepDelay += 50;

      else if (cmd == "]") armStepDelay = max(100, armStepDelay - 50);
      else if (cmd == "[") armStepDelay += 50;

      // ----------------------
      // SERVO COMMANDS
      // ----------------------
      else if (cmd.startsWith("A")) {
        int ang = cmd.substring(1).toInt();
        moveServoSmooth(baseA, ang + 4, 15);        // slower: 15ms per degree
        moveServoSmooth(baseB, (180 - ang) + 4, 15);  // mirrored
      }
      else if (cmd.startsWith("M")) {
        int ang = cmd.substring(1).toInt();
        moveServoSmooth(middleS, ang, 15);
      }
      else if (cmd.startsWith("W")) {
        int ang = cmd.substring(1).toInt();
        moveServoSmooth(wristS, ang, 15);
      }
      else if (cmd.startsWith("C")) {
        int ang = cmd.substring(1).toInt();
        moveServoSmooth(clawS, ang, 15);
      }
    }

    // Small delay to prevent task from hogging CPU
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ---------------------------------------------
// FREERTOS TASK: Stepper Motor Control
// ---------------------------------------------
void TaskStepperControl(void *pvParameters) {
  (void) pvParameters;

  while (1) {
    // ----------------------
    // STEPPER MOTION
    // ----------------------
    if (baseMoving) stepBase();
    if (armMoving) stepArm();

    // Yield to other tasks if no steppers are moving
    if (!baseMoving && !armMoving) {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
}

*/