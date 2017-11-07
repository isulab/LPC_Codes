

#include <LPC11xx.h>
#include "encoder.h"

int rotate0 = 0;		/* ���[�^���[�G���R�[�_(0��)�̒l���i�[ */
int rotate1 = 0;		/* ���[�^���[�G���R�[�_(1��)�̒l���i�[ */
int rotate2 = 0;		/* ���[�^���[�G���R�[�_(2��)�̒l���i�[ */

void RoryInit(void)
{
	rotate0 = rotate1 = rotate2 = 0;

	LPC_IOCON->PIO0_2 &= ~0x07;	/* 25�ԃs��(RORI0A)�̋@�\��PIO0_2�ɃN���A */
	LPC_IOCON->PIO0_2 |=  0x20;	/* 25�ԃs��(RORI0A)�̃q�X�e���V�X���C�l�[�u�� */
	LPC_IOCON->PIO0_3 &= ~0x07;	/* 26�ԃs��(RORI0B)�̋@�\��PIO0_3�ɃN���A */
	LPC_IOCON->PIO0_3 |=  0x20;	/* 26�ԃs��(RORI0B)�̃q�X�e���V�X���C�l�[�u�� */
	LPC_IOCON->PIO0_4 &= ~0x07;	/* 27�ԃs��(RORI1A)�̋@�\��PIO0_4�ɃN���A */
								/* I2C�Ƃ̌��p�s���Ńq�X�e���V�X�͂Ȃ� */
	LPC_IOCON->PIO0_5 &= ~0x07;	/* 5�ԃs��(RORI1B)�̋@�\��PIO0_5�ɃN���A */
								/* I2C�Ƃ̌��p�s���Ńq�X�e���V�X�͂Ȃ� */
	LPC_IOCON->PIO0_6 &= ~0x07;	/* 6�ԃs��(RORI2A)�̋@�\��PIO0_6�ɃN���A */
	LPC_IOCON->PIO0_6 |=  0x20;	/* 6�ԃs��(RORI2A)�̃q�X�e���V�X���C�l�[�u�� */
	LPC_IOCON->PIO0_7 &= ~0x07;	/* 28�ԃs��(RORI2B)�̋@�\��PIO0_7�ɃN���A */
	LPC_IOCON->PIO0_7 |=  0x20;	/* 28�ԃs��(RORI2B)�̃q�X�e���V�X���C�l�[�u�� */

	LPC_GPIO0->DIR	&= ~0x00fc;	/* PIO0_2����PIO0_11����͂ɐݒ� */

	LPC_GPIO0->IBE |= 0xFC;		/*���G�b�W���m*/
	LPC_GPIO0->IE  |= 0xFC;		/*�}�X�N����܂���*/

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


/* GPIO0�Ԃ̓��͊��荞�� */
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
