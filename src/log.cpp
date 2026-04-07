#include "arm.h"

void TaskLogging(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {

        uint16_t angle = encoder.readAngle();

        float degrees = angle * 360.0 / 4096.0;

        degrees += 137;
        if (degrees > 360)
        {
            degrees -= 360;
        }

        int baseAAngle = shoulderServoA.read();
        int baseBAngle = shoulderServoB.read();
        int middleAngle = wristServo.read();
        int wristAngle = twistServo.read();
        int clawAngle = clawServo.read();

        Serial.print("Encoder: ");
        Serial.print(degrees, 1);
        Serial.print("° | BaseA: ");
        Serial.print(baseAAngle);
        Serial.print("° | BaseB: ");
        Serial.print(baseBAngle);
        Serial.print("° | Middle: ");
        Serial.print(middleAngle);
        Serial.print("° | Wrist: ");
        Serial.print(wristAngle);
        Serial.print("° | Claw: ");
        Serial.print(clawAngle);
        Serial.println("°");

        // Serial.print("Angle: ");
        // Serial.print(degrees);
        // Serial.println(" deg-2");

        // int wristAngle = wristS.read();
        // Serial.print("Wrist Servo Angle: ");
        // Serial.print(wristAngle);
        // Serial.println(" deg");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}