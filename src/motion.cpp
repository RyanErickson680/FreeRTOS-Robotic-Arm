#include "arm.h"

#define IS_POSE_MODE 0

//---------------------------------------------
// High-level motion control task
//---------------------------------------------
void TaskMotionControl(void *pvParameters) {
  (void) pvParameters;
  PoseCommand target;

  for (;;) {
    // Receive latest pose (non-blocking, 100ms timeout)
    if (xQueueReceive(g_poseQueue, &target, 100 / portTICK_PERIOD_MS)) {
      // TODO: inverse kinematics with target.x, .y, .z, .roll, .pitch, .yaw
      //Serial.printf("Target: %.1f, %.1f, %.1f\n", target.x, target.y, target.z);
      Serial.print("Target: ");
        Serial.print(target.x);
        Serial.print(", ");
        Serial.print(target.y);
        Serial.print(", ");
        Serial.println(target.z);
      if (!IS_POSE_MODE) {
        struct JointAngles angles = {target.x, target.y, target.z, target.roll, target.pitch, target.yaw};
        xQueueOverwrite(g_anglesQueue, &angles);
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}