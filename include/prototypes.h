
extern QueueHandle_t g_poseQueue;
extern QueueHandle_t g_anglesQueue;

extern Servo baseA;
extern Servo baseB;
extern Servo middleS;
extern Servo wristS;
extern Servo clawS;
extern AS5600 encoder;

extern void TaskSerialCommands(void *pvParameters);
extern void TaskMotionControl(void *pvParameters);
extern void TaskMotorControl(void *pvParameters);
extern void TaskLogging(void *pvParameters);