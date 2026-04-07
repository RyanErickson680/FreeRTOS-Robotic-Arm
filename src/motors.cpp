#include "arm.h"

#include <pid.h>

#define CONTROL_RATE_MS 2
#define SERVO_SLEW_RATE_DPS 140.0f
#define SERVO_EASE_LINEAR_BLEND 0.35f
#define ELBOW_START_THRESHOLD_DEG 0.5f
#define ELBOW_STOP_THRESHOLD_DEG 0.15f
#define ELBOW_ERROR_FOR_MAX_SPEED_DEG 25.0f
#define ELBOW_SYNC_MIN_RUN_ERROR_DEG 0.03f
#define ELBOW_SYNC_ERROR_FOR_MAX_SPEED_DEG 8.0f
#define ELBOW_MIN_STEP_HZ 250.0f
#define ELBOW_MAX_STEP_HZ 10000.0f
#define ELBOW_SPEED_FILTER_ALPHA 0.60f
#define ELBOW_MAX_HZ_CHANGE_PER_S 32000.0f


IntervalTimer armTimer;
volatile bool armStepState = false;
volatile bool armRunning = false;

static float smoothstep01(float u) {
  u = constrain(u, 0.0f, 1.0f);
  const float eased = (u * u) * (3.0f - (2.0f * u));
  return eased + ((u - eased) * SERVO_EASE_LINEAR_BLEND);
}

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
  const float dt = CONTROL_RATE_MS / 1000.0f;
  const float servoNominalStep = SERVO_SLEW_RATE_DPS * dt;
  const float maxHzStepPerTick = ELBOW_MAX_HZ_CHANGE_PER_S * dt;

  PID_t armPID = { .kp = 0.2, .ki = 0.0, .kd = 0.0, .prevErr = 0.0, .integral = 0.0 };
  JointAngles_t targetAngles = {
    .base = 0.0f,
    .shoulder = 90.0f,
    .elbow = 180.0f,
    .wrist = 90.0f,
    .twist = 90.0f,
    .claw = 90.0f,
    .padding = 0.0f
  };
  JointAngles_t servoSetpoints = {
    .base = 0.0f,
    .shoulder = 90.0f,
    .elbow = 180.0f,
    .wrist = 90.0f,
    .twist = 90.0f,
    .claw = 90.0f,
    .padding = 0.0f
  };

  float elbowSetpoint = servoSetpoints.elbow;
  JointAngles_t motionStart = servoSetpoints;
  float elbowStart = elbowSetpoint;
  uint32_t syncTotalTicks = 0;
  uint32_t syncTick = 0;
  float elbowStepHzCommand = ELBOW_MIN_STEP_HZ;

  targetAngles.shoulder = servoSetpoints.shoulder;
  targetAngles.wrist = servoSetpoints.wrist;
  targetAngles.twist = servoSetpoints.twist;
  targetAngles.claw = servoSetpoints.claw;

  TickType_t lastWakeTime = xTaskGetTickCount();

  for(;;) {
    // Check for new target angles
    if (xQueueReceive(g_anglesQueue, &targetAngles, 0) == pdPASS) {
      Serial.printf("%.1f %.1f %.1f %.1f %.1f %.1f\n",
                    targetAngles.base, targetAngles.shoulder, targetAngles.elbow,
                    targetAngles.wrist, targetAngles.twist, targetAngles.claw);

      // Inverse kinematics calculates angles from -180 to 180
      // We need to shift each angle, and adjust any servos that
      // use 270 degree range by scaling them into 180 degrees
      targetAngles.base = constrain(targetAngles.base, MIN_BASE_ANGLE, MAX_BASE_ANGLE);
      targetAngles.shoulder = constrain(180.0f - targetAngles.shoulder, MIN_SHOULDER_ANGLE, MAX_SHOULDER_ANGLE);
      targetAngles.elbow = constrain(180.0f - targetAngles.elbow , MIN_ELBOW_ANGLE, MAX_ELBOW_ANGLE);
      targetAngles.wrist = constrain((135.0f + targetAngles.wrist) * (2.0/3.0), MIN_WRIST_ANGLE, MAX_WRIST_ANGLE);
      targetAngles.twist = constrain(targetAngles.twist + 90, MIN_TWIST_ANGLE, MAX_TWIST_ANGLE);
      targetAngles.claw = constrain(targetAngles.claw, MIN_CLAW_ANGLE, MAX_CLAW_ANGLE);

      const float shoulderDelta = targetAngles.shoulder - servoSetpoints.shoulder;
      const float wristDelta = targetAngles.wrist - servoSetpoints.wrist;
      const float twistDelta = targetAngles.twist - servoSetpoints.twist;
      const float clawDelta = targetAngles.claw - servoSetpoints.claw;
      const float elbowDelta = targetAngles.elbow - elbowSetpoint;

      // speed is based on joint that has to move the most
      const float maxDelta = fmaxf(
        fmaxf(fabsf(shoulderDelta), fabsf(wristDelta)),
        fmaxf(fmaxf(fabsf(twistDelta), fabsf(clawDelta)), fabsf(elbowDelta))
      );

      if (maxDelta < 0.001f) {
        syncTotalTicks = 0;
        syncTick = 0;
        servoSetpoints.shoulder = targetAngles.shoulder;
        servoSetpoints.wrist = targetAngles.wrist;
        servoSetpoints.twist = targetAngles.twist;
        servoSetpoints.claw = targetAngles.claw;
        elbowSetpoint = targetAngles.elbow;
      } else {
        // calculate how many ticks to get to new position
        const uint32_t totalTicks = (uint32_t)ceilf(maxDelta / servoNominalStep);
        syncTotalTicks = (totalTicks == 0u) ? 1u : totalTicks;
        syncTick = 0;
        motionStart = servoSetpoints;
        elbowStart = elbowSetpoint;
      }

      // Serial.printf("New angles: Base=%.1f Shoulder=%.1f Elbow=%.1f Wrist=%.1f Twist=%.1f Claw=%.1f\n",
      //               targetAngles.base, targetAngles.shoulder, targetAngles.elbow,
      //               targetAngles.wrist, targetAngles.twist, targetAngles.claw);
      // Serial.printf("%.1f %.1f %.1f %.1f %.1f %.1f\n",
      //               targetAngles.base, targetAngles.shoulder, targetAngles.elbow,
      //               targetAngles.wrist, targetAngles.twist, targetAngles.claw);

    }

    if (syncTick < syncTotalTicks) {
      syncTick++;
      const float u = (float)syncTick / (float)syncTotalTicks;
      const float s = smoothstep01(u);

      // intermediate setpoints for smoother motion
      servoSetpoints.shoulder = motionStart.shoulder + ((targetAngles.shoulder - motionStart.shoulder) * s);
      servoSetpoints.wrist = motionStart.wrist + ((targetAngles.wrist - motionStart.wrist) * s);
      servoSetpoints.twist = motionStart.twist + ((targetAngles.twist - motionStart.twist) * s);
      servoSetpoints.claw = motionStart.claw + ((targetAngles.claw - motionStart.claw) * s);
      elbowSetpoint = elbowStart + ((targetAngles.elbow - elbowStart) * s);

      if (syncTick >= syncTotalTicks) {
        // make sure setpoint is exact target on last tick
        servoSetpoints.shoulder = targetAngles.shoulder;
        servoSetpoints.wrist = targetAngles.wrist;
        servoSetpoints.twist = targetAngles.twist;
        servoSetpoints.claw = targetAngles.claw;
        elbowSetpoint = targetAngles.elbow;
      }
    }

    twistServo.write(servoSetpoints.twist);
    clawServo.write(servoSetpoints.claw);
    wristServo.write(servoSetpoints.wrist);
    shoulderServoA.write(servoSetpoints.shoulder + 4);
    shoulderServoB.write((180 - servoSetpoints.shoulder) + 4);  // mirrored


    //targetAngles.arm = 270.0f;

    // --------------------------------------------
    // Stepper controls
    // --------------------------------------------

    uint16_t angle = encoder.readAngle();

        float degrees = angle * 360.0 / 4096.0;

        degrees += 137;
        if (degrees > 360)
        {
            degrees -= 360;
        }
    error = PID_Update(&armPID, elbowSetpoint, degrees);

    const float absError = fabsf(error);

    const bool syncActive = (syncTick < syncTotalTicks);
    const float runThreshold = syncActive
      ? ELBOW_SYNC_MIN_RUN_ERROR_DEG
      : (armRunning ? ELBOW_STOP_THRESHOLD_DEG : ELBOW_START_THRESHOLD_DEG);
    const bool shouldMove = absError > runThreshold;

    if (!shouldMove) {
      if (armRunning) {
        stopArmMotor();
      }
      elbowStepHzCommand = ELBOW_MIN_STEP_HZ;
    } else {
      digitalWrite(ARM_DIR_PIN, error < 0 ? HIGH : LOW);

      const float errorForMaxSpeed = syncActive ? ELBOW_SYNC_ERROR_FOR_MAX_SPEED_DEG : ELBOW_ERROR_FOR_MAX_SPEED_DEG;
      const float errorRatio = constrain(absError / errorForMaxSpeed, 0.0f, 1.0f);
      const float targetStepHz = ELBOW_MIN_STEP_HZ + ((ELBOW_MAX_STEP_HZ - ELBOW_MIN_STEP_HZ) * errorRatio);

      const float speedFilterAlpha = syncActive ? 0.85f : ELBOW_SPEED_FILTER_ALPHA;
      const float maxHzDelta = syncActive ? (maxHzStepPerTick * 2.0f) : maxHzStepPerTick;
      const float requestedDelta = speedFilterAlpha * (targetStepHz - elbowStepHzCommand);
      elbowStepHzCommand += constrain(requestedDelta, -maxHzDelta, maxHzDelta);
      elbowStepHzCommand = constrain(elbowStepHzCommand, ELBOW_MIN_STEP_HZ, ELBOW_MAX_STEP_HZ);

      period = (uint32_t)(500000.0f / elbowStepHzCommand);
      period = constrain(period, (uint32_t)100, (uint32_t)2000);

      if (!armRunning) {
        startArmMotor(period);
      } else {
        armTimer.update(period);
      }

    }
    
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CONTROL_RATE_MS));
  }

}
