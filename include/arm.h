#ifndef ARM_H
#define ARM_H



#include <Arduino.h>
#include <Servo.h>
#include <FreeRTOS.h>
#include <task.h>
#include <Wire.h>
#include <AS5600.h>
#include <queue.h>

#include "pins.h"
#include "prototypes.h"

struct PoseCommand {
  float x, y, z, roll, pitch, yaw;
};

struct JointAngles {
  float base, arm, middle, wrist, twist, claw;
};

#endif // ARM_H