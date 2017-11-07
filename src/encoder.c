

#include <LPC11xx.h>
#include "encoder.h"

int rotate0 = 0;		/* ロータリーエンコーダ(0番)の値を格納 */
int rotate1 = 0;		/* ロータリーエンコーダ(1番)の値を格納 */
int rotate2 = 0;		/* ロータリーエンコーダ(2番)の値を格納 */

void RoryInit(void)
{
	rotate0 = rotate1 = rotate2 = 0;

	LPC_IOCON->PIO0_2 &= ~0x07;	/* 25番ピン(RORI0A)の機能をPIO0_2にクリア */
	LPC_IOCON->PIO0_2 |=  0x20;	/* 25番ピン(RORI0A)のヒステリシスをイネーブル */
	LPC_IOCON->PIO0_3 &= ~0x07;	/* 26番ピン(RORI0B)の機能をPIO0_3にクリア */
	LPC_IOCON->PIO0_3 |=  0x20;	/* 26番ピン(RORI0B)のヒステリシスをイネーブル */
	LPC_IOCON->PIO0_4 &= ~0x07;	/* 27番ピン(RORI1A)の機能をPIO0_4にクリア */
								/* I2Cとの兼用ピンでヒステリシスはない */
	LPC_IOCON->PIO0_5 &= ~0x07;	/* 5番ピン(RORI1B)の機能をPIO0_5にクリア */
								/* I2Cとの兼用ピンでヒステリシスはない */
	LPC_IOCON->PIO0_6 &= ~0x07;	/* 6番ピン(RORI2A)の機能をPIO0_6にクリア */
	LPC_IOCON->PIO0_6 |=  0x20;	/* 6番ピン(RORI2A)のヒステリシスをイネーブル */
	LPC_IOCON->PIO0_7 &= ~0x07;	/* 28番ピン(RORI2B)の機能をPIO0_7にクリア */
	LPC_IOCON->PIO0_7 |=  0x20;	/* 28番ピン(RORI2B)のヒステリシスをイネーブル */

	LPC_GPIO0->DIR	&= ~0x00fc;	/* PIO0_2からPIO0_11を入力に設定 */

	LPC_GPIO0->IBE |= 0xFC;		/*両エッジ検知*/
	LPC_GPIO0->IE  |= 0xFC;		/*マスクされません*/

	NVIC_SetPriority(EINT0_IRQn,8);
	NVIC_EnableIRQ(EINT0_IRQn);
	return ;
}

/*

0 0000  0
1 0001  1
2 0010 -1
3 0011  0
4 0100 -1
5 0101  0
6 0110  0
7 0111  1
8 1000  1
9 1001  0
A 1010  0
B 1011 -1
C 1100  0
D 1101 -1
E 1110  1
F 1111  0

*/

void readENC0(void)
{
	const int enctable[]={0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};
	unsigned static int last = 0;

	rotate0 += enctable[last = ((last<<2)|(LPC_GPIO0->DATA>>2)&3)&0x0f];
	return;
}

void readENC1(void)
{
	const int enctable[]={0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};
	unsigned static int last = 0;

	rotate1 += enctable[last = ((last<<2)|(LPC_GPIO0->DATA>>4)&3)&0x0f];
	return;
}

void readENC2(void)
{
	const int enctable[]={0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};
	unsigned static int last = 0;

	rotate2 += enctable[last = ((last<<2)|(LPC_GPIO0->DATA>>6)&3)&0x0f];
	return;
}


/* GPIO0番の入力割り込み */
void PIOINT0_IRQHandler(void)
{
	if(LPC_GPIO0->MIS & 0x04)
	{
		readENC0();
		LPC_GPIO0->IC |= 0x04;
	}
	else if(LPC_GPIO0->MIS & 0x08)
	{
		readENC0();
		LPC_GPIO0->IC |= 0x08;
	}
	else if(LPC_GPIO0->MIS & 0x10)
	{
		readENC1();
		LPC_GPIO0->IC |= 0x10;
	}
	else if(LPC_GPIO0->MIS & 0x20)
	{
		readENC1();
		LPC_GPIO0->IC |= 0x20;
	}
	else if(LPC_GPIO0->MIS & 0x40)
	{
		readENC2();
		LPC_GPIO0->IC |= 0x40;
	}
	else if(LPC_GPIO0->MIS & 0x80)
	{
		readENC2();
		LPC_GPIO0->IC |= 0x80;
	}

	__asm volatile ("nop");
	__asm volatile ("nop");
}
