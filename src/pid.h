
typedef struct{
	double kp;				/* ”ä—á¬•ª‚ÌŒW” */
	double ki;				/* Ï•ª¬•ª‚ÌŒW” */
	double kd;				/* ”÷•ª¬•ª‚ÌŒW” */
	double integral;		/* Ï•ª¬•ª‚ÌÏZ•”•ª */
	int olddeff;			/* ‘O‰ñ‚Ì· */
	int targetValue;		/* –Ú•W’l */
	int currentValue;		/* Œ»İ’l */
	int operationAmount;	/* ‘€ì—Ê */
}pid_t;

void PIDStructClear(pid_t* pidstruct);

void pid_calc(pid_t* pid);

extern pid_t Motor0_PIDStruct;
extern pid_t Motor1_PIDStruct;
extern pid_t Motor2_PIDStruct;
