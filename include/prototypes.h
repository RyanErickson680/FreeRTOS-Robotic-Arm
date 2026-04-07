
extern QueueHandle_t g_poseQueue;
extern QueueHandle_t g_anglesQueue;

extern Servo shoulderServoA;
extern Servo shoulderServoB;
extern Servo wristServo;
extern Servo twistServo;
extern Servo clawServo;
extern AS5600 encoder;

/* Tasks */
extern void TaskSerialCommands(void *pvParameters);
extern void TaskMotionSupervisor(void *pvParameters);
extern void TaskMotorControl(void *pvParameters);
extern void TaskLogging(void *pvParameters);

extern bool Arm_InverseKinematics(const struct PoseCommand *p, struct JointAngles *out);