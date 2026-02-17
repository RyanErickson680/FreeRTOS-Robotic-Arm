#include "pid.h"

float PID_Update(PID_t *pid, float set, float meas)
{
    float err = set - meas;

    pid->integral += err;
    float deriv = err - pid->prevErr;

    pid->prevErr = err;

    return pid->kp * err +
           pid->ki * pid->integral +
           pid->kd * deriv;
}