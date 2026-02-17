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
        xQueueOverwrite(g_poseQueue, &cmd);
      } else {
        Serial.println("Invalid command received");
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}