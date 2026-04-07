#include "arm.h"
//#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline float DegToRad(float deg) {
    return deg * (float)(M_PI / 180.0);
}

static inline float RadToDeg(float rad) {
    return rad * (float)(180.0 / M_PI);
}


/*
 * Solves IK for 5-DOF arm + claw
 *
 * Returns false if unreachable
 */
bool Arm_InverseKinematics(const PoseCommand_t *p,
                           JointAngles_t *out)
{
    /* -------- Inputs -------- */

    float x = p->x;
    float y = p->y;
    float z = p->z;

    float roll  = p->roll;
    float pitch = p->pitch;

    // Input orientation is provided in degrees.
    float pitchRad = DegToRad(pitch);

    /* -------- Base rotation -------- */

    float q1 = atan2f(y, x);
    

    /* -------- Project to arm plane -------- */

    float r = sqrtf(x*x + y*y);

    /* -------- Wrist center --------
       Remove tool length from target
    */

    float wx = r - L3 * cosf(pitchRad);
    float wz = z - L3 * sinf(pitchRad);

    /* -------- Reachability -------- */

    float D =
        (wx*wx + wz*wz - L1*L1 - L2*L2) /
        (2.0f * L1 * L2);

    // Keep strict reject for clearly unreachable targets,
    // but tolerate tiny floating-point spillover at the boundary.
    if (D < -1.001f || D > 1.001f)
        return false;   // Outside workspace

    D = constrain(D, -1.0f, 1.0f);

    /* -------- Elbow  -------- */

    float q3 = atan2f(
        -1 * sqrtf(1.0f - D*D),   // elbow-up
        D
    );

    /* -------- Shoulder -------- */

    float q2 =
        atan2f(wz, wx) -
        atan2f(L2 * sinf(q3),
               L1 + L2 * cosf(q3));

    /* -------- Wrist pitch -------- */

    float q4 = pitchRad - q2 - q3;

    /* -------- Wrist roll -------- */

    float q5 = roll;

    /* -------- Output -------- */

    // convert to degrees
    q1 = RadToDeg(q1);
    q2 = RadToDeg(q2);
    q3 = RadToDeg(q3);
    q4 = RadToDeg(q4);
    q5 = RadToDeg(q5);

    // Normalize base angle to [0, 360)
    if (q1 < 0.0f) {
        q1 += 360.0f;
    }

    out->base = q1;
    out->shoulder = q2;
    out->elbow = q3;
    out->wrist = q4;
    out->twist = q5;


    out->claw = p->grip;

    return true;
}