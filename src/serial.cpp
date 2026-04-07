// serial.cpp
#include "arm.h"


//---------------------------------------------
// FREERTOS TASK: Serial Command Processing
//---------------------------------------------
void TaskSerialCommands(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    while (Serial.available() >= 24) {
      struct PoseCommand cmd;
      if (Serial.readBytes((char *)&cmd, sizeof(cmd)) == sizeof(cmd)) {
        //Serial.printf("Received command: x=%.1f y=%.1f z=%.1f roll=%.1f pitch=%.1f yaw=%.1f grip=%.1f\n",
          //            cmd.x, cmd.y, cmd.z, cmd.roll, cmd.pitch, cmd.yaw, cmd.grip);
        xQueueOverwrite(g_poseQueue, &cmd);
      } else {
        Serial.println("Invalid command received");
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}