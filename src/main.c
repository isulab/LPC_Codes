

#include <LPC11xx.h>
#include "motor.h"
#include "encoder.h"
#include "uart.h"
#include "pid.h"


int main(void)
{
	int count;

	PIDStructClear(&Motor0_PIDStruct);
	PIDStructClear(&Motor1_PIDStruct);
	PIDStructClear(&Motor2_PIDStruct);

	Motor0_PIDStruct.kp = 0.8;
	Motor1_PIDStruct.kp = 0.8;
	Motor2_PIDStruct.kp = 0.8;
	Motor0_PIDStruct.ki = 0.0008;
	Motor1_PIDStruct.ki = 0.0008;
	Motor2_PIDStruct.ki = 0.0008;
	Motor0_PIDStruct.kd = 40.;
	Motor1_PIDStruct.kd = 40.;
	Motor2_PIDStruct.kd = 40.;

	MotorInit();
	RoryInit();
	uartInit();

	for(count = 0;;++count){
		if(operationIOHex){
			putRotate();
			msecWait(10);
		}
		else{
			uart_putint(count,6);
			if(operation)
				uart_puts(": (Operation ) ");
			else if(initiation)
				uart_puts(": (Initiation) ");
			else if(UpMode)
				uart_puts(": (Up        ) ");
			else if(Kattsun)
				uart_puts(": (Kattsun   ) ");
			else
				uart_puts(": (          ) ");
			uart_puts("Rotate=(");
			uart_putint(rotate0,6);
			uart_puts(", ");
			uart_putint(rotate1,6);
			uart_puts(", ");
			uart_putint(rotate2,6);
			uart_puts(")");

			if(operation){
				uart_puts(", duty =(");
				uart_putint(Motor0_PIDStruct.operationAmount,6);
				uart_puts(", ");
				uart_putint(Motor1_PIDStruct.operationAmount,6);
				uart_puts(", ");
				uart_putint(Motor2_PIDStruct.operationAmount,6);
				uart_puts("), ");
				uart_putint(INtargetM0Edge,6);
				uart_puts(", ");
				uart_putint(INtargetM1Edge,6);
				uart_puts(", ");
				uart_putint(INtargetM2Edge,6);
			}
			else if(Kattsun){
				uart_puts(", duty =(");
				uart_putint(KattsunM0duty,6);
				uart_puts(", ");
				uart_putint(KattsunM1duty,6);
				uart_puts(", ");
				uart_putint(KattsunM2duty,6);
				uart_puts(") ");
			}
			uart_puts("\r\n");
			msecWait(250);
		}


	}

	for(;;);

	return 0;
}
