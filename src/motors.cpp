#include "arm.h"

#include <pid.h>

#define CONTROL_RATE_MS 2

void TaskMotorControl(void *pvParameters) {
  (void) pvParameters;

  PID_t armPID = { .kp = 1.0, .ki = 0.0, .kd = 0.0, .prevErr = 0.0, .integral = 0.0 };
  TickType_t lastWakeTime = xTaskGetTickCount();
  struct JointAngles targetAngles;

  for(;;) {
    // Check for new target angles
    if (xQueueReceive(g_anglesQueue, &targetAngles, 0) == pdPASS) {
      Serial.printf("New angles: Base=%.1f Arm=%.1f Middle=%.1f Wrist=%.1f Twist=%.1f Claw=%.1f\n",
                    targetAngles.base, targetAngles.arm, targetAngles.middle,
                    targetAngles.wrist, targetAngles.twist, targetAngles.claw);

        
  }

  PID_Update(&armPID, targetAngles.arm, encoder.readAngle());

  vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CONTROL_RATE_MS));
  }

}

void stepArm(uint32_t speed) {
  digitalWrite(ARM_STEP_PIN, HIGH);
  delayMicroseconds(speed);
  digitalWrite(ARM_STEP_PIN, LOW);
  delayMicroseconds(speed);
}