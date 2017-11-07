
#define TARGET_INCREMENT 4
#define INCREMENT_DIVISION 33

void TIMER16_0_Init(void);
void TIMER32_1_Init(void);
void TIMER16_1_Init(void);
void MotorInit(void);

void Motor0_drive(int duty);
void Motor1_drive(int duty);
void Motor2_drive(int duty);

void msecWait(unsigned int time);

extern int PIDtargetM0Edge;
extern int PIDtargetM1Edge;
extern int PIDtargetM2Edge;

typedef struct{
	int max_speed;
	int acceleration;
	int current_speed;
	int deceleration_count_up;
	int deceleration_count_down;
	int deceleration_pattern[TARGET_INCREMENT];
}AccelerationSuppressionStruct_t;

extern AccelerationSuppressionStruct_t accelSup_M0;
extern AccelerationSuppressionStruct_t accelSup_M1;
extern AccelerationSuppressionStruct_t accelSup_M2;

void AccelerationSuppression_init(int speed_in, int acceleration_in, AccelerationSuppressionStruct_t* AccelerationSuppressionStruct);
int AccelerationSuppression(int current_position, int target, AccelerationSuppressionStruct_t* conf);
