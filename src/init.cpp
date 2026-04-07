/* init.cpp - setup() */
#include "arm.h"

#define ENABLE_LOGGING 0

// ---------------------------------------------
// GLOBAL VARIABLES
// ---------------------------------------------
// Stepper parameters
int baseStepDelay = 500;
int armStepDelay = 500;

bool baseMoving = false;
bool armMoving = false;

int baseDirection = LOW;
int armDirection = LOW;

// Servos
Servo shoulderServoA;
Servo shoulderServoB;
Servo wristServo;
Servo twistServo;
Servo clawServo;

// AS5600 mag encoder
AS5600 encoder;

QueueHandle_t g_poseQueue = NULL;
QueueHandle_t g_anglesQueue = NULL;

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
// SETUP
// ---------------------------------------------
void setup()
{
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
    while (1)
    {
        if (!encoder.begin())
        {
            Serial.println("AS5600 not detected!");
        }
        else
        {
            break;
        }
    }

    digitalWrite(BASE_EN_PIN, LOW); // enable drivers
    digitalWrite(ARM_EN_PIN, LOW);

    // Attach servos
    shoulderServoA.attach(BASE_SERVO_A);
    shoulderServoB.attach(BASE_SERVO_B);
    wristServo.attach(MIDDLE_SERVO);
    twistServo.attach(WRIST_SERVO);
    clawServo.attach(CLAW_SERVO);

    int ang = 90;

    // shoulderServoA.write(ang + 4);
    // shoulderServoB.write((180 - ang) + 4);  // mirrored    
    // twistServo.write(ang);
    // wristServo.write(ang);
    // clawServo.write(ang);

    delay(1000);


    g_poseQueue = xQueueCreate(1, sizeof(PoseCommand));
    g_anglesQueue = xQueueCreate(1, sizeof(JointAngles));

    Serial.begin(115200);
    Serial.println("Robotic Arm Controller Ready");
    

    // Create FreeRTOS tasks
    xTaskCreate(
        TaskSerialCommands,
        "SerialCmd",
        8192, // Stack size
        NULL,
        2, // Priority
        NULL);
    
    xTaskCreate(
        TaskMotionSupervisor,
        "MotionCtrl",
        8192, // Stack size
        NULL,
        3, // Priority
        NULL);

    xTaskCreate(
        TaskMotorControl,
        "MotorCtrl",
        8192, // Stack size
        NULL,
        5, // Priority
        NULL);

    if (ENABLE_LOGGING) {
        xTaskCreate(
            TaskLogging,
            "Logging",
            8192, // Stack size
            NULL,
            2, // Priority
            NULL);
    }

    // Start the scheduler
    vTaskStartScheduler();
}

void loop() {}