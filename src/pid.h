
typedef struct{
	double kp;				/* ��ᐬ���̌W�� */
	double ki;				/* �ϕ������̌W�� */
	double kd;				/* ���������̌W�� */
	double integral;		/* �ϕ������̐ώZ���� */
	int olddeff;			/* �O��̍� */
	int targetValue;		/* �ڕW�l */
	int currentValue;		/* ���ݒl */
	int operationAmount;	/* ����� */
}pid_t;

void PIDStructClear(pid_t* pidstruct);

void pid_calc(pid_t* pid);

extern pid_t Motor0_PIDStruct;
extern pid_t Motor1_PIDStruct;
extern pid_t Motor2_PIDStruct;
