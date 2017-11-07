
#include <LPC11xx.h>
#include "uart.h"
#include "encoder.h"
#include "pid.h"
#include "motor.h"

double dabs(double n)
{
	return n>0?n:-n;
}

void baudrateInit(unsigned long int baudrate)
{	const int MCLK = 48000000;
	int PCLK = MCLK / LPC_SYSCON->UARTCLKDIV;

	int div_dlest;
	int div_divaddval=2;
	int div_mulval=1;
	int div_dlm;
	int div_dll;
	double div_frest;
	int i,j;

	LPC_UART->LCR |= 0x80;		/* ボーレート設定へのアクセスをイネーブル */

	div_dlest=PCLK/(baudrate*16);
	if(!(PCLK%(baudrate*16))){
		div_divaddval=0;
		div_mulval   =1;
	}
	else{
		div_frest = 1.5;
		div_dlest = PCLK/(16*baudrate*div_frest);
		div_frest = (double)PCLK/(16*baudrate*div_dlest);

		for(i=1;i<16;i++){
			for(j=0;j<i;j++){
				if(dabs(div_frest-(1.+(double)j/i))<dabs(div_frest-(1.+(double)div_divaddval/div_mulval))){
					div_mulval=i;
					div_divaddval=j;
				}
			}
		}
	}
	div_dlm=(div_dlest>>8)&0xff;
	div_dll=(div_dlest)&0xff;

	LPC_UART->DLM = div_dlm;
	LPC_UART->DLL = div_dll;
	LPC_UART->FDR &= ~0x000000FF;
	LPC_UART->FDR |= div_divaddval | div_mulval<<4 ;

	LPC_UART->LCR &= ~0x80;			/* ボーレート設定へのアクセスをディスエーブル */

	return ;
}

void uartInit(void)
{
	LPC_SYSCON->SYSAHBCLKCTRL |= 1<<12;
	LPC_SYSCON->UARTCLKDIV = 4;
	LPC_IOCON->PIO1_6 |= 0x01;		/* 15番ピンをRXDに設定 */
	LPC_IOCON->PIO1_7 |= 0x01;		/* 16番ピンをTXDに設定 */

	LPC_UART->LCR |= 0x03|0x80;		/* 8ビット文字長 */
	baudrateInit(38400);			/* ボーレートを38400に設定 */

	LPC_UART->IER |= 3;				/* RDA割り込みとTHRE割り込みを有効 */
	LPC_UART->FCR = 0x07;			/* FIFOを有効 */

	NVIC_SetPriority(UART_IRQn,4);
	NVIC_EnableIRQ(UART_IRQn);

	return ;
}

unsigned char instr[64]="";
int rxf=1;

void uart_putchar(unsigned char c)
{
    while(!(LPC_UART->LSR & (1<<5)));
    LPC_UART->THR = c;
    return ;
}

void uart_puts(unsigned char* s)
{
	while(*s)uart_putchar(*s++);
	return ;
}

unsigned char uart_getchar(void)
{
	while(!(LPC_UART->LSR & 0x01));
	return  LPC_UART->RBR;
}

void uart_putuint(unsigned int num, unsigned int digit)
{
	unsigned char str??(16??)="              0";
	int p=14;

	if(digit>15)digit=15;
	while((num) && (p>=0))
	{
		str[p--]=(num%10)+'0';
		num/=10;
	}
	uart_puts(str+(15-digit));
}

void uart_putint(int num, unsigned int digit)
{
	unsigned char str??(16??)="              0";
	int p=14;
	int minus=0;

	if(num < 0)
	{
		num = -num;
		minus = 1;
	}

	if(digit>15)digit=15;

	while((num) && (p>=0))
	{
		str[p--]=(num%10)+'0';
		num/=10;
	}

	if(minus)str[p] = '-';
	uart_puts(str+(15-digit));
}

int operation = 0;
int operationIOHex = 0;
int initiation = 0;
int UpMode = 0;
int Kattsun = 0;
int sinWave = 0;
int sinpos = 0;

unsigned int INtargetM0Edge = 0;
unsigned int INtargetM1Edge = 0;
unsigned int INtargetM2Edge = 0;

int KattsunM0duty = 0;
int KattsunM1duty = 0;
int KattsunM2duty = 0;

unsigned int hextoi(unsigned char c)
{
	if('0'<=c&&c<='9')return c-'0';
	if('a'<=c&&c<='f')return c-'a'+10;
	if('A'<=c&&c<='F')return c-'A'+10;
	return 0;
}

unsigned char itoHex(unsigned int i){
	if(0x0<=i&&i<=0x9)return i+'0';
	if(0xa<=i&&i<=0xf)return i+'A'-10;
	return '0';
}

unsigned char itohex(unsigned int i){
	if(0x0<=i&&i<=0x9)return i+'0';
	if(0xa<=i&&i<=0xf)return i+'a'-10;
	return '0';
}

int settargetfd(unsigned char* str){
	int num=0;
	int i;

	if(!('0'<=str[0] && str[0] <= '2')) return -1;

	for(i=1;'0'<=str[i] && str[i]<='9';++i){
		num *= 10;
		num += str[i]-'0';
	}

	if(str[i]!='\r')return -1;
	if(str[++i]!='\n')return -1;

	if(str[0]=='0')INtargetM0Edge = num;
	if(str[0]=='1')INtargetM1Edge = num;
	if(str[0]=='2')INtargetM2Edge = num;

	return 0;
}

int settarget(unsigned char* str)
{
	int i;
	for(i=0;i<12;i++){
		if(!('0'<=str[i]&&str[i]<='9' || 'a'<=str[i]&&str[i]<='f' || 'A'<=str[i]&&str[i]<='F'))return -1;
	}
	if(str[12]!='\r')return -1;
	if(str[13]!='\n')return -1;
	INtargetM0Edge = (hextoi(str[ 0])<<12) + (hextoi(str[ 1])<<8) + (hextoi(str[ 2])<<4) + (hextoi(str[ 3]));
	INtargetM1Edge = (hextoi(str[ 4])<<12) + (hextoi(str[ 5])<<8) + (hextoi(str[ 6])<<4) + (hextoi(str[ 7]));
	INtargetM2Edge = (hextoi(str[ 8])<<12) + (hextoi(str[ 9])<<8) + (hextoi(str[10])<<4) + (hextoi(str[11]));
	return 0;
}

int putRotate(void){
	uart_putchar(itoHex((rotate0>>12)&0xf));
	uart_putchar(itoHex((rotate0>>8)&0xf));
	uart_putchar(itoHex((rotate0>>4)&0xf));
	uart_putchar(itoHex((rotate0)&0xf));
	uart_putchar(itoHex((rotate1>>12)&0xf));
	uart_putchar(itoHex((rotate1>>8)&0xf));
	uart_putchar(itoHex((rotate1>>4)&0xf));
	uart_putchar(itoHex((rotate1)&0xf));
	uart_putchar(itoHex((rotate2>>12)&0xf));
	uart_putchar(itoHex((rotate2>>8)&0xf));
	uart_putchar(itoHex((rotate2>>4)&0xf));
	uart_putchar(itoHex((rotate2)&0xf));
	uart_putchar('\r');
	uart_putchar('\n');

	return 0;
}

int setduty(unsigned char* str)
{
	int num=0;
	int i;

//	uart_puts("setduty\r\n");

	if(!('0'<=str[0] && str[0] <= '2')) return -1;

//	uart_puts("head\r\n");

	if(str[1]=='-'){
		for(i=2;'0'<=str[i] && str[i]<='9';++i){
			num *= 10;
			num += str[i]-'0';
		}
		num *= -1;
	}
	else{
		for(i=1;'0'<=str[i] && str[i]<='9';++i){
			num *= 10;
			num += str[i]-'0';
		}
	}

	if(str[i]!='\r')return -1;
	if(str[++i]!='\n')return -1;

	if(str[0]=='0')KattsunM0duty = num;
	if(str[0]=='1')KattsunM1duty = num;
	if(str[0]=='2')KattsunM2duty = num;

//	uart_puts("return\r\n");

	return 0;
}

void UART_IRQHandler(void)
{
	char c;
	static unsigned char inbuf[64] = "";
	static unsigned char outbuf[64] = "";
	static unsigned char* inbufp = inbuf;
	static unsigned char* outbufp = outbuf;
	unsigned long int* cpyp1;
	unsigned long int* cpyp2;
	int i;

	if(LPC_UART->IIR&0x04){
		c=uart_getchar();
		if(c=='s' || c=='S' || c==' '){
			Kattsun = operation = initiation = UpMode = sinWave = 0;
			operationIOHex = 0;
			Motor0_PIDStruct.integral = 0.;
			Motor1_PIDStruct.integral = 0.;
			Motor2_PIDStruct.integral = 0.;
			Motor0_PIDStruct.olddeff  = 0;
			Motor1_PIDStruct.olddeff  = 0;
			Motor2_PIDStruct.olddeff  = 0;

			inbufp = inbuf;
		}
		else if(c=='o' || c=='O' || c=='p' || c=='P'){
			Motor0_PIDStruct.integral = 0.;
			Motor1_PIDStruct.integral = 0.;
			Motor2_PIDStruct.integral = 0.;
			Motor0_PIDStruct.olddeff  = 0;
			Motor1_PIDStruct.olddeff  = 0;
			Motor2_PIDStruct.olddeff  = 0;

			PIDtargetM0Edge = INtargetM0Edge = rotate0;
			PIDtargetM1Edge = INtargetM1Edge = rotate1;
			PIDtargetM2Edge = INtargetM2Edge = rotate2;

			accelSup_M0.current_speed = accelSup_M0.deceleration_count_down = accelSup_M0.deceleration_count_up = 0;
			accelSup_M1.current_speed = accelSup_M1.deceleration_count_down = accelSup_M1.deceleration_count_up = 0;
			accelSup_M2.current_speed = accelSup_M2.deceleration_count_down = accelSup_M2.deceleration_count_up = 0;


			if(c=='o' || c=='O')operationIOHex = 1;
			else operationIOHex = 0;

			Kattsun = initiation = UpMode = sinWave = 0;
			operation = 1;
			inbufp = inbuf;
		}
		else if(c=='i' || c=='I'){
			Kattsun = UpMode = operation = sinWave = 0;
			operationIOHex = 0;
			initiation = 1;
			inbufp = inbuf;
		}
		else if(c=='u' || c=='U'){
			Kattsun = initiation = operation = sinWave = 0;
			operationIOHex = 0;
			UpMode = 1;
			inbufp = inbuf;
		}
		else if(c=='k' || c== 'K'){
			KattsunM0duty = KattsunM1duty = KattsunM2duty = 0;
			operation = initiation = UpMode = sinWave = 0;
			operationIOHex = 0;
			Kattsun = 1;
			inbufp = inbuf;
		}
/*		else if(c=='g' || c== 'G'){
			Motor0_PIDStruct.integral = 0.;
			Motor0_PIDStruct.olddeff  = 0;
			PIDtargetM0Edge = INtargetM0Edge = rotate0;
			PIDtargetM1Edge = INtargetM1Edge = rotate1;
			PIDtargetM2Edge = INtargetM2Edge = rotate2;

			operationIOHex = 0;

			sinpos = 0;

			sinWave = 1;
			Kattsun = initiation = UpMode = 0;
			operation = 1;
		}*/
		else if(c=='\n'){
			*inbufp++ = c;
			*inbufp = 0;
			cpyp1=inbuf;
			cpyp2=instr;
			for(i=0;i<16;++i){
				*cpyp2++=*cpyp1++;
			}
			inbufp = inbuf;

			if(operation && !sinWave){
				if(operationIOHex)settarget(instr);
				else settargetfd(instr);
			}
			else if(Kattsun)setduty(instr);

			rxf=1;
		}
		else if(c==0x1b){/* Escape */
			inbufp = inbuf;
		}
		else if(c==0x08){/* BackSpacce */
			if(inbufp>inbuf)--inbufp;
		}

		else{
			*inbufp++ = c;
		}
	}
}
