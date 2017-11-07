

#include <LPC11xx.h>
#include <math.h>
#include "motor.h"
#include "uart.h"
#include "pid.h"
#include "encoder.h"

void TIMER16_0_Init(void)
{
	LPC_IOCON->PIO0_8               &= ~0x07;		/* 1�ԃs���̐ݒ��PIO0_8�ɃN���A */
	LPC_IOCON->PIO0_9               &= ~0x07;		/* 2�ԃs���̐ݒ��PIO0_9�ɃN���A */
	LPC_IOCON->JTAG_TCK_PIO0_10     &= ~0x07;		/* 3�ԃs���̐ݒ��SWCLK�ɃN���A  */
	LPC_IOCON->PIO0_8               |=  0x02;		/* 1�ԃs����CT16B0_MAT0�ɐݒ� */
	LPC_IOCON->PIO0_9               |=  0x02;		/* 2�ԃs����CT16B0_MAT1�ɐݒ� */
	LPC_IOCON->JTAG_TCK_PIO0_10     |=  0x03;		/* 3�ԃs����CT16B0_MAT2�ɐݒ� */

	LPC_SYSCON->SYSAHBCLKCTRL       |= (1<<7);		/* �N���b�N������ */
	LPC_TMR16B0->PR                  =  40 - 1;		/* 48M /  8 =  6MHz */
	LPC_TMR16B0->MR3                 = 256 - 1;		/* 6M  / 256 = 23.4375kHz */
	LPC_TMR16B0->MCR                 = (1<<10);		/* MR3=TC��TC�����Z�b�g */
	LPC_TMR16B0->MR0                 = 256;			/* PWM�o��0�̃f���[�e�B�[���0%�� */
	LPC_TMR16B0->MR1                 = 256;			/* PWM�o��1�̃f���[�e�B�[���0%�� */
	LPC_TMR16B0->MR2                 = 256;			/* PWM�o��1�̃f���[�e�B�[���0%�� */
	LPC_TMR16B0->PWMC                =   7;			/* CT16B0_MAT0��CT16B0_MAT1,CT16B0_MAT2��PWM�o�� */

	LPC_TMR16B0->TCR                 =   1;			/* �J�E���g�J�n */

	return ;
}

void TIMER32_1_Init(void)
{
	LPC_IOCON->JTAG_TDO_PIO1_1      &= ~0x07;		/* 10�ԃs���̐ݒ��R�ɃN���A */
	LPC_IOCON->JTAG_nTRST_PIO1_2    &= ~0x07;		/* 11�ԃs���̐ݒ��R�ɃN���A */
	LPC_IOCON->ARM_SWDIO_PIO1_3     &= ~0x07;		/* 12�ԃs���̐ݒ��SWDIO�ɃN���A */
	LPC_IOCON->JTAG_TDO_PIO1_1      |=  0x03;		/* 10�ԃs����CT32B1_MAT0�ɐݒ� */
	LPC_IOCON->JTAG_nTRST_PIO1_2    |=  0x03;		/* 11�ԃs����CT32B1_MAT1�ɐݒ� */
	LPC_IOCON->ARM_SWDIO_PIO1_3     |=  0x03;		/* 12�ԃs����CT32B1_MAT2�ɐݒ� */

	LPC_SYSCON->SYSAHBCLKCTRL   |= (1<<10);		/* �N���b�N������ */
	LPC_TMR32B1->PR             =  40 - 1;		/* 48M /  8 =  6MHz */
	LPC_TMR32B1->MR3            = 256 - 1;		/* 6M  / 256 = 23.4375kHz */
	LPC_TMR32B1->MCR            = (1<<10);		/* MR3=TC��TC�����Z�b�g */
	LPC_TMR32B1->MR0            = 256;			/* PWM�o��0�̃f���[�e�B�[���0%�� */
	LPC_TMR32B1->MR1            = 256;			/* PWM�o��1�̃f���[�e�B�[���0%�� */
	LPC_TMR32B1->MR2            = 256;			/* PWM�o��2�̃f���[�e�B�[���0%�� */
	LPC_TMR32B1->PWMC           =   7;			/* CT32B1_MAT0��CT32B1_MAT1,CT32B1_MAT2��PWM�o�� */

	LPC_TMR32B1->TCR            =   1;			/* �J�E���g�J�n */

	return ;
}

void TIMER16_1_Init(void)
{
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<8);	/* �N���b�N������ */
	LPC_TMR16B1->PR   =  48 - 1;			/* 48M /   48 =  1MHz */
	LPC_TMR16B1->MR3  =  1000 - 1;			/*  1M / 1000 =  1kHz */
	LPC_TMR16B1->MCR  |= (3<<9);			/* MR3=TC�Ŋ��荞�ݗv��,TC�����Z�b�g */
	NVIC_EnableIRQ(TIMER_16_1_IRQn);		/* ���荞�݋��� */
	NVIC_SetPriority(TIMER_16_1_IRQn,2);	/* ���荞�ݗD��x��ݒ� */
	LPC_TMR16B1->TCR  =   1;				/* �J�E���g�J�n */

	return ;
}

void MotorInit(void)
{
	LPC_IOCON->PIO1_8			&= ~0x07;	/* 17�ԃs��(M0_AH)�̋@�\��PIO1_8�ɃN���A */
	LPC_IOCON->PIO1_9			&= ~0x07;	/* 18�ԃs��(M0_BH)�̋@�\��PIO1_9�ɃN���A */
	LPC_IOCON->JTAG_TMS_PIO1_0	&= ~0x07;	/* 9�ԃs��(M1_AH)�̋@�\��R�ɃN���A */
	LPC_IOCON->JTAG_TMS_PIO1_0	|=  0x01;	/* 9�ԃs��(M1_AH)�̋@�\��PIO1_0�ɃZ�b�g */
	LPC_IOCON->PIO1_4			&= ~0x07;	/* 13�ԃs��(M1_BH)�̋@�\��PIO1_4�ɃN���A */
	LPC_IOCON->JTAG_TDI_PIO0_11 &= ~0x07;	/* 4�ԃs��(M2_AH)�̋@�\��R�ɃN���A */
	LPC_IOCON->JTAG_TDI_PIO0_11 |=  0x01;	/* 4�ԃs��(M2_AH)�̋@�\��PIO0_11�ɃZ�b�g */
	LPC_IOCON->PIO1_5			&= ~0x07;	/* 14�ԃs��(M2_BH)�̋@�\��PIO1_5�ɃN���A */

	LPC_GPIO0->DIR				|= 0x0800;	/* ��L��6�s�����o�͂ɐݒ� */
	LPC_GPIO1->DIR				|= 0x0331;	/* ��L��6�s�����o�͂ɐݒ� */

	TIMER16_0_Init();
	TIMER32_1_Init();
	TIMER16_1_Init();

	AccelerationSuppression_init(TARGET_INCREMENT, INCREMENT_DIVISION, &accelSup_M0);
	AccelerationSuppression_init(TARGET_INCREMENT, INCREMENT_DIVISION, &accelSup_M1);
	AccelerationSuppression_init(TARGET_INCREMENT, INCREMENT_DIVISION, &accelSup_M2);

	return ;
}

static int motor0_dutyPlus;
static int motor0_dutyMinus;
static int motor1_dutyPlus;
static int motor1_dutyMinus;
static int motor2_dutyPlus;
static int motor2_dutyMinus;

void Motor0_drive(int duty)
{
	if(duty> 256)duty= 256;
	if(duty<-256)duty=-256;

	if(duty<0){
		motor0_dutyMinus = 0;
		motor0_dutyPlus  = -duty;

		return ;
	}
	motor0_dutyPlus  = 0;
	motor0_dutyMinus = duty;

	return ;
}

void Motor1_drive(int duty)
{
	if(duty> 256)duty= 256;
	if(duty<-256)duty=-256;

	if(duty<0){
		motor1_dutyMinus = 0;
		motor1_dutyPlus  = -duty;

		return ;
	}
	motor1_dutyPlus  = 0;
	motor1_dutyMinus = duty;

	return ;
}

void Motor2_drive(int duty)
{
	if(duty> 256)duty= 256;
	if(duty<-256)duty=-256;

	if(duty<0){
		motor2_dutyMinus = 0;
		motor2_dutyPlus  = -duty;

		return ;
	}
	motor2_dutyPlus  = 0;
	motor2_dutyMinus = duty;

	return ;
}

static int M0q1On = 10;
static int M0q2On = 10;
static int M1q1On = 10;
static int M1q2On = 10;
static int M2q1On = 10;
static int M2q2On = 10;

/*
 * 7[ms]�Ńf�b�h�^�C�����K�v
 * �}�[�W�������10[ms]
 */

unsigned int waittime = 0;

void msecWait(unsigned int time)
{
	waittime = time;
	while(waittime);
	return ;
}

int PIDtargetM0Edge = 0;
int PIDtargetM1Edge = 0;
int PIDtargetM2Edge = 0;

AccelerationSuppressionStruct_t accelSup_M0;
AccelerationSuppressionStruct_t accelSup_M1;
AccelerationSuppressionStruct_t accelSup_M2;

/*
 * �����̃p�^�[��������
 * n       : �ڕW�ʒu�Ƃ̍�
 * pattern : �p�^�[��
 * size    : pattern�̃T�C�Y
 */
int searchPattern(int n, int* pattern, int size)
{
	int i;
	if(n<0)n=0;
	for(i=0; i<size; i++){
		if(n<pattern[i])return i+1;
	}
	return i;
}

/*
 * ��`���䂷��֐�
 * current   : ���݂̑��x
 * target    : �ڕW�̑��x
 * division  : ����ڂ̌Ăяo����1�������邩
 * counter   : �J�E���^
 * �Ԃ�l     : �ڕW�̈ʒu
 */
int accelsup(int current, int target, int division, int* counter_up, int* counter_down)
{
	if(division>0)--division;
	if(current>target){
		if(!*counter_down){
			*counter_down=division;
			return current-1;
		}
	}
	else if(current<target){
		if(!*counter_up){
			*counter_up=division;
			return current+1;
		}
	}
	if(*counter_down) --(*counter_down);
	if(*counter_up) --(*counter_up);
	return current;
}

/*
 * ��`���䂷��֐�
 * current   : ���݂̈ʒu
 * target    : �ڕW�̈ʒu
 * increment : ���x
 * �Ԃ�l     : �ڕW�̈ʒu
 */
int trapezoid(int current, int target, int increment){
	if(current>target){
		if(current-target <= increment) return target;
		return current - increment;
	}

	if(target-current <= increment) return target;
	return current + increment;
}

/*
 * speed_in        : �ő�̑���[Edge/ms]
 * acceleration_in : ��[ms]��1[Edge/ms]�������邩
 */
void AccelerationSuppression_init(int speed_in, int acceleration_in, AccelerationSuppressionStruct_t* AccelerationSuppressionStruct)
{
	int i;

	for(i=0, AccelerationSuppressionStruct->deceleration_pattern[i] = acceleration_in*(i+1); ++i<speed_in;){
		AccelerationSuppressionStruct->deceleration_pattern[i] = AccelerationSuppressionStruct->deceleration_pattern[i-1] + acceleration_in*(i+1);
	}

	AccelerationSuppressionStruct->max_speed = speed_in;
	AccelerationSuppressionStruct->acceleration = acceleration_in;
	AccelerationSuppressionStruct->current_speed = 0;
	AccelerationSuppressionStruct->deceleration_count_down = 0;
	AccelerationSuppressionStruct->deceleration_count_up = 0;

	return ;
}

int AccelerationSuppression(int current_position, int target, AccelerationSuppressionStruct_t* conf){
//	uart_putint(target,4);
//	uart_puts(" ");
	target = trapezoid(current_position,target,searchPattern(abs(current_position - target)-abs(conf->current_speed),conf->deceleration_pattern,conf->max_speed));
//	uart_putint(target,4);
//	uart_puts(" ");
	target = current_position + (conf->current_speed=accelsup(conf->current_speed, target - current_position, conf->acceleration, &(conf->deceleration_count_down), &(conf->deceleration_count_up)));
//	uart_putint(target,4);
//u	uart_puts("\r\n");
	return target;
}

void TIMER16_1_IRQHandler(void)
{
	if(waittime)--waittime;

	if(sinWave){
		INtargetM0Edge = sin((double)sinpos/5000.*2.*3.141592653589793238462643383280)*500+2000;
		INtargetM1Edge = sin((double)sinpos/5000.*2.*3.141592653589793238462643383280+2.0943951023931954923084289221863)*500+2000;
		INtargetM2Edge = sin((double)sinpos/5000.*2.*3.141592653589793238462643383280-2.0943951023931954923084289221863)*500+2000;
		if(++sinpos >= 5000)sinpos = 0;
	}

	if(operation){
		PIDtargetM0Edge = AccelerationSuppression(PIDtargetM0Edge,INtargetM0Edge,&accelSup_M0);
		PIDtargetM1Edge = AccelerationSuppression(PIDtargetM1Edge,INtargetM1Edge,&accelSup_M1);
		PIDtargetM2Edge = AccelerationSuppression(PIDtargetM2Edge,INtargetM2Edge,&accelSup_M2);

		Motor0_PIDStruct.currentValue = rotate0;
		Motor1_PIDStruct.currentValue = rotate1;
		Motor2_PIDStruct.currentValue = rotate2;
		Motor0_PIDStruct.targetValue  = PIDtargetM0Edge;
		Motor1_PIDStruct.targetValue  = PIDtargetM1Edge;
		Motor2_PIDStruct.targetValue  = PIDtargetM2Edge;
		pid_calc(&Motor0_PIDStruct);
		pid_calc(&Motor1_PIDStruct);
		pid_calc(&Motor2_PIDStruct);
		Motor0_drive(Motor0_PIDStruct.operationAmount);
		Motor1_drive(Motor1_PIDStruct.operationAmount);
		Motor2_drive(Motor2_PIDStruct.operationAmount);
	}
	else{
		Motor0_PIDStruct.integral = 0.;
		Motor1_PIDStruct.integral = 0.;
		Motor2_PIDStruct.integral = 0.;

		if(Kattsun){
			Motor0_drive(KattsunM0duty);
			Motor1_drive(KattsunM1duty);
			Motor2_drive(KattsunM2duty);
		}
		else if(initiation){
			Motor0_drive(-50);
			Motor1_drive(-50);
			Motor2_drive(-50);
			rotate0 = 0;
			rotate1 = 0;
			rotate2 = 0;
		}
		else if(UpMode){
			Motor0_drive(50);
			Motor1_drive(50);
			Motor2_drive(50);
		}
		else{
			Motor0_drive(0);
			Motor1_drive(0);
			Motor2_drive(0);
		}
	}

	if(motor0_dutyPlus  && !motor0_dutyMinus && LPC_TMR16B0->MR1>255)
		LPC_GPIO1->DATA |=  0x100;
	else
		LPC_GPIO1->DATA &= ~0x100;

	if(motor0_dutyMinus && !motor0_dutyPlus  && LPC_TMR16B0->MR0>255)
		LPC_GPIO1->DATA |=  0x200;
	else
		LPC_GPIO1->DATA &= ~0x200;

	if(motor1_dutyPlus  && !motor1_dutyMinus && LPC_TMR32B1->MR1>255)
		LPC_GPIO1->DATA |=  0x001;
	else
		LPC_GPIO1->DATA &= ~0x001;

	if(motor1_dutyMinus && !motor1_dutyPlus  && LPC_TMR32B1->MR0>255)
		LPC_GPIO1->DATA |=  0x010;
	else
		LPC_GPIO1->DATA &= ~0x010;

	if(motor2_dutyPlus  && !motor2_dutyMinus && LPC_TMR32B1->MR2>255)
		LPC_GPIO0->DATA |=  0x800;
	else
		LPC_GPIO0->DATA &= ~0x800;

	if(motor2_dutyMinus && !motor2_dutyPlus  && LPC_TMR16B0->MR2>255)
		LPC_GPIO1->DATA |=  0x020;
	else
		LPC_GPIO1->DATA &= ~0x020;

	if(LPC_GPIO1->DATA & 0x100) M0q1On = 10;
	if(LPC_GPIO1->DATA & 0x200) M0q2On = 10;
	if(LPC_GPIO1->DATA & 0x001) M1q1On = 10;
	if(LPC_GPIO1->DATA & 0x010) M1q2On = 10;
	if(LPC_GPIO0->DATA & 0x800) M2q1On = 10;
	if(LPC_GPIO1->DATA & 0x020) M2q2On = 10;

	if(M0q1On)	--M0q1On;
	else		LPC_TMR16B0->MR1 = 256 - motor0_dutyMinus;
	if(M0q2On)	--M0q2On;
	else		LPC_TMR16B0->MR0 = 256 - motor0_dutyPlus;
	if(M1q1On)	--M1q1On;
	else		LPC_TMR32B1->MR1 = 256 - motor1_dutyMinus;
	if(M1q2On)	--M1q2On;
	else		LPC_TMR32B1->MR0 = 256 - motor1_dutyPlus;
	if(M2q1On)	--M2q1On;
	else		LPC_TMR32B1->MR2 = 256 - motor2_dutyMinus;
	if(M2q2On)	--M2q2On;
	else		LPC_TMR16B0->MR2 = 256 - motor2_dutyPlus;/**/

	LPC_TMR16B1->IR |= 0x08;
	return ;
}
