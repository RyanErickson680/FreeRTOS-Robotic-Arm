#include "arm.h"

#define IS_POSE_MODE 1

//---------------------------------------------
// High-level motion control task
//---------------------------------------------
void TaskMotionSupervisor(void *pvParameters) {
  (void) pvParameters;
  PoseCommand_t targetPose;
  JointAngles_t angles;

  for (;;) {
    // Receive latest pose (non-blocking, 100ms timeout)
    if (xQueueReceive(g_poseQueue, &targetPose,  pdMS_TO_TICKS(100))) {

      if (IS_POSE_MODE) {
        // perform IK
        bool ok = Arm_InverseKinematics(&targetPose, &angles);
        if (ok) {
          xQueueOverwrite(g_anglesQueue, &angles);
        }
        else {
          Serial.println("no IK solution");
        }

      } else {
        // control via direct angle assignment (instead of IK)
        angles = {targetPose.x, targetPose.y, targetPose.z, targetPose.roll, targetPose.pitch, targetPose.yaw};
        xQueueOverwrite(g_anglesQueue, &angles);
      }
    }
    //vTaskDelay(50 / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}