#pragma once

/* Link lengths (mm) */
#define L1  185.0f   // Shoulder -> Elbow
#define L2  117.5f   // Elbow -> Wrist
//#define L3   86.0f   // Wrist -> Tool tip (before claw)
#define L3   236.0f   // Wrist -> Tool tip (after claw, for IK calculations)

typedef struct PoseCommand{
    float x;
    float y;
    float z;

    float roll;
    float pitch;
    float yaw;

    float grip;
} PoseCommand_t;

typedef struct JointAngles {
  float base;
  float shoulder;
  float elbow;
  float wrist;
  float twist;
  float claw;

  float padding; // extra float to match size of PoseCommand for interchangeability
} JointAngles_t;