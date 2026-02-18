#include "arm.h"

#include <pid.h>

#define CONTROL_RATE_MS 2


IntervalTimer armTimer;
volatile bool armStepState = false;
volatile bool armRunning = false;


// Function to be ran by the timer
void stepArmISR() {
  armStepState = !armStepState;
  digitalWriteFast(ARM_STEP_PIN, armStepState);
}

void startArmMotor(uint32_t period_us) {
  armTimer.begin(stepArmISR, period_us);
  armRunning = true;
}

void stopArmMotor() {
  armTimer.end();
  digitalWriteFast(ARM_STEP_PIN, LOW);
  armRunning = false;
}

void TaskMotorControl(void *pvParameters) {
  (void) pvParameters;

  float error = 0.0;
  uint32_t period;

  PID_t armPID = { .kp = 2.0, .ki = 0.0, .kd = 0.0, .prevErr = 0.0, .integral = 0.0 };
  struct JointAngles targetAngles;

  TickType_t lastWakeTime = xTaskGetTickCount();

  for(;;) {
    // Check for new target angles
    if (xQueueReceive(g_anglesQueue, &targetAngles, 0) == pdPASS) {
      Serial.printf("New angles: Base=%.1f Arm=%.1f Middle=%.1f Wrist=%.1f Twist=%.1f Claw=%.1f\n",
                    targetAngles.base, targetAngles.arm, targetAngles.middle,
                    targetAngles.wrist, targetAngles.twist, targetAngles.claw);

        
    }


    targetAngles.arm = 270.0f;

    uint16_t angle = encoder.readAngle();

        float degrees = angle * 360.0 / 4096.0;

        degrees += 137;
        if (degrees > 360)
        {
            degrees -= 360;
        }

    error = PID_Update(&armPID, targetAngles.arm, degrees);
    
    float absError = fabs(error);

    if (absError < 0.5f) {
      if (armRunning) {
        stopArmMotor();
      }
    } else {
      digitalWrite(ARM_DIR_PIN, error < 0 ? HIGH : LOW);

      period = 1000 - (absError);
      period = constrain(period, (uint32_t)100, (uint32_t)2000);

      if (!armRunning) {
        // run at slower speed incredibly briefly before using PID output
        startArmMotor(2000);
      } else {
        armTimer.update(period);
      }

    }
    
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CONTROL_RATE_MS));
  }

}
