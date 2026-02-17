#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float prevErr;
    float integral;
} PID_t;

float PID_Update(PID_t *pid, float set, float meas);

#endif // PID_H