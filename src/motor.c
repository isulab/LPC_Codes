

#include <LPC11xx.h>
#include <math.h>
#include "motor.h"
#include "uart.h"
#include "pid.h"
#include "encoder.h"

void TIMER16_0_Init(void)
{
	LPC_IOCON->PIO0_8               &= ~0x07;		/* 1番ピンの設定をPIO0_8にクリア */
	LPC_IOCON->PIO0_9               &= ~0x07;		/* 2番ピンの設定をPIO0_9にクリア */
	LPC_IOCON->JTAG_TCK_PIO0_10     &= ~0x07;		/* 3番ピンの設定をSWCLKにクリア  */
	LPC_IOCON->PIO0_8               |=  0x02;		/* 1番ピンをCT16B0_MAT0に設定 */
	LPC_IOCON->PIO0_9               |=  0x02;		/* 2番ピンをCT16B0_MAT1に設定 */
	LPC_IOCON->JTAG_TCK_PIO0_10     |=  0x03;		/* 3番ピンをCT16B0_MAT2に設定 */

	LPC_SYSCON->SYSAHBCLKCTRL       |= (1<<7);		/* クロックを供給 */
	LPC_TMR16B0->PR                  =  40 - 1;		/* 48M /  8 =  6MHz */
	LPC_TMR16B0->MR3                 = 256 - 1;		/* 6M  / 256 = 23.4375kHz */
	LPC_TMR16B0->MCR                 = (1<<10);		/* MR3=TCでTCをリセット */
	LPC_TMR16B0->MR0                 = 256;			/* PWM出力0のデューティー比を0%に */
	LPC_TMR16B0->MR1                 = 256;			/* PWM出力1のデューティー比を0%に */
	LPC_TMR16B0->MR2                 = 256;			/* PWM出力1のデューティー比を0%に */
	LPC_TMR16B0->PWMC                =   7;			/* CT16B0_MAT0とCT16B0_MAT1,CT16B0_MAT2をPWM出力 */

	LPC_TMR16B0->TCR                 =   1;			/* カウント開始 */

	return ;
}

void TIMER32_1_Init(void)
{
	LPC_IOCON->JTAG_TDO_PIO1_1      &= ~0x07;		/* 10番ピンの設定をRにクリア */
	LPC_IOCON->JTAG_nTRST_PIO1_2    &= ~0x07;		/* 11番ピンの設定をRにクリア */
	LPC_IOCON->ARM_SWDIO_PIO1_3     &= ~0x07;		/* 12番ピンの設定をSWDIOにクリア */
	LPC_IOCON->JTAG_TDO_PIO1_1      |=  0x03;		/* 10番ピンをCT32B1_MAT0に設定 */
	LPC_IOCON->JTAG_nTRST_PIO1_2    |=  0x03;		/* 11番ピンをCT32B1_MAT1に設定 */
	LPC_IOCON->ARM_SWDIO_PIO1_3     |=  0x03;		/* 12番ピンをCT32B1_MAT2に設定 */

	LPC_SYSCON->SYSAHBCLKCTRL   |= (1<<10);		/* クロックを供給 */
	LPC_TMR32B1->PR             =  40 - 1;		/* 48M /  8 =  6MHz */
	LPC_TMR32B1->MR3            = 256 - 1;		/* 6M  / 256 = 23.4375kHz */
	LPC_TMR32B1->MCR            = (1<<10);		/* MR3=TCでTCをリセット */
	LPC_TMR32B1->MR0            = 256;			/* PWM出力0のデューティー比を0%に */
	LPC_TMR32B1->MR1            = 256;			/* PWM出力1のデューティー比を0%に */
	LPC_TMR32B1->MR2            = 256;			/* PWM出力2のデューティー比を0%に */
	LPC_TMR32B1->PWMC           =   7;			/* CT32B1_MAT0とCT32B1_MAT1,CT32B1_MAT2をPWM出力 */

	LPC_TMR32B1->TCR            =   1;			/* カウント開始 */

	return ;
}

void TIMER16_1_Init(void)
{
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<8);	/* クロックを供給 */
	LPC_TMR16B1->PR   =  48 - 1;			/* 48M /   48 =  1MHz */
	LPC_TMR16B1->MR3  =  1000 - 1;			/*  1M / 1000 =  1kHz */
	LPC_TMR16B1->MCR  |= (3<<9);			/* MR3=TCで割り込み要求,TCをリセット */
	NVIC_EnableIRQ(TIMER_16_1_IRQn);		/* 割り込み許可 */
	NVIC_SetPriority(TIMER_16_1_IRQn,2);	/* 割り込み優先度を設定 */
	LPC_TMR16B1->TCR  =   1;				/* カウント開始 */

	return ;
}

void MotorInit(void)
{
	LPC_IOCON->PIO1_8			&= ~0x07;	/* 17番ピン(M0_AH)の機能をPIO1_8にクリア */
	LPC_IOCON->PIO1_9			&= ~0x07;	/* 18番ピン(M0_BH)の機能をPIO1_9にクリア */
	LPC_IOCON->JTAG_TMS_PIO1_0	&= ~0x07;	/* 9番ピン(M1_AH)の機能をRにクリア */
	LPC_IOCON->JTAG_TMS_PIO1_0	|=  0x01;	/* 9番ピン(M1_AH)の機能をPIO1_0にセット */
	LPC_IOCON->PIO1_4			&= ~0x07;	/* 13番ピン(M1_BH)の機能をPIO1_4にクリア */
	LPC_IOCON->JTAG_TDI_PIO0_11 &= ~0x07;	/* 4番ピン(M2_AH)の機能をRにクリア */
	LPC_IOCON->JTAG_TDI_PIO0_11 |=  0x01;	/* 4番ピン(M2_AH)の機能をPIO0_11にセット */
	LPC_IOCON->PIO1_5			&= ~0x07;	/* 14番ピン(M2_BH)の機能をPIO1_5にクリア */

	LPC_GPIO0->DIR				|= 0x0800;	/* 上記の6ピンを出力に設定 */
	LPC_GPIO1->DIR				|= 0x0331;	/* 上記の6ピンを出力に設定 */

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
 * 7[ms]でデッドタイムが必要
 * マージン取って10[ms]
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
 * 減速のパターンを検索
 * n       : 目標位置との差
 * pattern : パターン
 * size    : patternのサイズ
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
 * 台形制御する関数
 * current   : 現在の速度
 * target    : 目標の速度
 * division  : 何回目の呼び出しで1減速するか
 * counter   : カウンタ
 * 返り値     : 目標の位置
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
 * 台形制御する関数
 * current   : 現在の位置
 * target    : 目標の位置
 * increment : 速度
 * 返り値     : 目標の位置
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
 * speed_in        : 最大の速さ[Edge/ms]
 * acceleration_in : 何[ms]で1[Edge/ms]減速するか
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
