#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "uart.h"
#include "string.h"
#include "hw_ints.h"

#define SYSTICK_FREQUENCY		1000			//1000hz

void 		S800_GPIO_Init(void);
void 		S800_UART_Init(void);
void UARTStringPut(const char *cMessage);

uint32_t ui32SysClock;
char RxBuf[256];
int RxEndFlag=0;
char month[12][4]={"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};

int main(void)
{
	char a[4];
	int monthnumber=-1,monthnumber2=-1,i;
	//use internal 16M oscillator, PIOSC
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		
	 //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 8000000);		
	//use external 25M oscillator, MOSC
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_OSC), 25000000);		

	//use external 25M oscillator and PLL to 120M
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);;		
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 20000000);
	
	S800_GPIO_Init();
	S800_UART_Init();
	
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
  IntEnable(INT_UART0);
	
  //SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY);//1ms
	//SysTickIntEnable();
  IntMasterEnable();		
  //SysTickEnable();

	while (1)
	{
		if (RxEndFlag)
		{
			strncpy(a,RxBuf,3);
			a[3]='\0';
			for( i=0;i<12;i++)
			{
				if(strcmp(a,month[i])==0) {monthnumber=i;break;}
			}
			if(monthnumber!=-1)
			{
				if(RxBuf[4]<='9' && RxBuf[4]>='0' && RxBuf[5]<='9' && RxBuf[5]>='0')
				{
					monthnumber2=10*(RxBuf[4]-'0')+RxBuf[5]-'0';					
				}
				if(RxBuf[3]=='+' && monthnumber2!=-1)
				{ 
					monthnumber+=monthnumber2;
					while(monthnumber>11) monthnumber-=12;
					UARTStringPut(month[monthnumber]);
		      UARTStringPut("\r\n");//回车换行
				}
				else if(RxBuf[3]=='-' && monthnumber2!=-1)
				{				
					monthnumber-=monthnumber2;
					while(monthnumber<0) monthnumber+=12;
					UARTStringPut(month[monthnumber]);
		      UARTStringPut("\r\n");//回车换行
				}
			}
		  RxEndFlag=0; 
			monthnumber2=-1;
		}
	}
}

void UARTStringPut(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPut(UART0_BASE,*(cMessage++));
}

void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PA1_U0TX);    			

  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX7_8);
	UARTStringPut("\r\nHello, world!\r\n");
}
void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);						//Enable PortN	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)){};			//Wait for the GPIO moduleN ready		
	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);			//Set PN0 as Output pin

	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

void UART0_Handler(void) 
{ 
	uint8_t cnt = 0;
  uint32_t ulStatus;
  ulStatus = UARTIntStatus(UART0_BASE, true); 
  UARTIntClear(UART0_BASE, ulStatus); 
	
  while( UARTCharsAvail(UART0_BASE) )
		{ 
			RxBuf[cnt++]= UARTCharGetNonBlocking(UART0_BASE);
		}
  RxBuf[cnt]='\0'; 
  RxEndFlag=1; 
}

