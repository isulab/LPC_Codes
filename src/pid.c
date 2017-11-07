#include "pid.h"

pid_t Motor0_PIDStruct;
pid_t Motor1_PIDStruct;
pid_t Motor2_PIDStruct;

void pid_calc(pid_t* pid)
{
	int p;
	int i;
	int d;

	p = pid->targetValue - pid->currentValue;
	d = p - pid->olddeff;
	pid->olddeff = p;
	i = (pid->integral += p);

	pid->operationAmount = p*pid->kp + i*pid->ki + d*pid->kd;
}

void PIDStructClear(pid_t* pidstruct)
{
	pidstruct->currentValue=0;
	pidstruct->integral = 0.;
	pidstruct->kd = 0.;
	pidstruct->ki = 0.;
	pidstruct->kp = 0.;
	pidstruct->olddeff = 0;
	pidstruct->operationAmount = 0;
	pidstruct->targetValue = 0;
}
