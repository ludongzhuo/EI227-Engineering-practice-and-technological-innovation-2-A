#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
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
#include <string.h>
#include "hw_ints.h"
#include "timer.h"
#include "pwm.h"

#define SYSTICK_FREQUENCY		1000			//1000Hz
#define	I2C_FLASHTIME				500			  //500ms

//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 					0x22
#define PCA9557_I2CADDR						0x18

#define PCA9557_INPUT							0x00
#define	PCA9557_OUTPUT						0x01
#define PCA9557_POLINVERT					0x02
#define PCA9557_CONFIG						0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06


void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);
void    S800_PWM_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void		S800_I2C0_Init(void);
void 		S800_UART_Init(void);
void    S800_Timer0_Init(void);
void    UARTStringPut(const char *cMessage);
void    S800_Beep(int n);

void Show_student_number(void);//显示学号
void Serial_port_function(void);//实现串行口功能
void Time_display(void);//时间显示
void Date_display(void);//日期显示
void Time_setting(void);//时间设置
void Date_setting(void);//日期设置
void Alarm1_setting(void);//闹钟1设置
void Alarm2_setting(void);//闹钟2设置
void Time_adjustment(void);//处理时间各数字的进位退位
void Date_adjustment(void);//处理日期各数字的进位退位
void Alarm1_adjustment(void);//处理闹钟1各数字的进位退位
void Alarm2_adjustment(void);//处理闹钟2各数字的进位退位

//systick software counter define
volatile uint16_t systick_10ms_couter,systick_1s_couter,systick_100ms_couter;
volatile uint8_t	systick_10ms_status,systick_1s_status,systick_100ms_status;
uint32_t cnt;
volatile uint8_t result,key_value,i2c_status;
volatile uint8_t rightshift = 0x01;
uint32_t ui32SysClock;
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};

char RxBuf[256];
int RxEndFlag=0;

bool pressed1,pressed2,pressed3,pressed4,pressed5,pressed6,pressed7,pressed8,pressed;//防抖用的标志

int hour=23,min=58,sec=45,centi_sec=78;//时间
int month=6,day=20;//日期
int monthdays[13]={31,31,29,31,30,31,30,31,31,30,31,30,31};//12月、1月、2月……12月的天数
int hour_alarm1=7,min_alarm1=0,hour_alarm2=13,min_alarm2=30;//闹钟1、2的小时、分钟
bool alarm1_ok,alarm2_ok;//闹钟1、2是否有效（值为1代表有效）
bool alarm1_ring,alarm2_ring;//闹钟1、2是否到时（值为1代表闹钟响了）
bool wake=1;
int Function=0;//功能序号(0为时间显示，1为日期显示，2为时间设置，3为日期设置，4为闹钟1设置，5为闹钟2设置)
uint8_t num;//显示数码管不同位置
uint8_t set_status=0;//记录此时是在设置时间或日期中的哪一位
uint8_t twelvehours_status=0,pm_status=0;//值为1分别表示在12小时制和处于pm
int hour_process(int num)
{
	if(twelvehours_status) return num%12;
	else return num;
}
uint8_t pm_i2c()
{
	if(pm_status) return 0xbf;
	else return 0xff;
}

int main(void)
{
	//use internal 16M oscillator, PIOSC
  ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		

	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	S800_Timer0_Init();
	
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
  IntEnable(INT_UART0);
	
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT );
	IntEnable(INT_TIMER0A);
	
  SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY);//1ms
	SysTickIntEnable();
  IntMasterEnable();		
  SysTickEnable();
	
	Show_student_number();
	
	while (1)
	{
		if (RxEndFlag)
    { 
			Serial_port_function();
			RxEndFlag=0; 
		}
		switch(Function)
		{
			case 0:Time_display();break;	
			case 1:Date_display();break;	
		  case 2:Time_setting();break;	
			case 3:Date_setting();break;
			case 4:Alarm1_setting();break;
			case 5:Alarm2_setting();break;
		}
		if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfe) pressed1=true;
	  else{		if(pressed1)	{	pressed1=false;	Function++;	}		}
		if(Function>5) Function=0;
		
		if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xef) pressed5=true;
	  else{		if(pressed5)	{	pressed5=false;	alarm1_ok=!alarm1_ok;	}		}
		if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xdf) pressed6=true;
	  else{		if(pressed6)	{	pressed6=false;	alarm2_ok=!alarm2_ok;	}		}
		if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xbf) pressed7=true;
	  else{		if(pressed7)	{	pressed7=false;	twelvehours_status=!twelvehours_status;	}		}
	  
		if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)!=0xff &&(alarm1_ring||alarm2_ring)){//闹钟到时按键失能
			pressed8=true;
			pressed1=pressed2=pressed3=pressed4=pressed5=pressed6=pressed7=false;
		}
		else{	
			if(pressed8){
				pressed1=pressed2=pressed3=pressed4=pressed5=pressed6=pressed7=false;
				if(alarm1_ring) alarm1_ok=false;
				if(alarm2_ring) alarm2_ok=false;
				pressed8=false;
			}
		}
		if(alarm1_ring) S800_Beep(1);if(alarm2_ring) S800_Beep(2); else GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_5,0);//闹钟结束后置零
		
		if (systick_100ms_status)
		{
			systick_100ms_status	= 0;
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xff) cnt++;
			else cnt=0;
		}
		
		if (cnt	>= 100)//如果超过10秒不按键，则返回时间显示
		{	cnt	= 0;Function=0;	}
		
		if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0) pressed=true;
	  else{		if(pressed)	{	pressed=false;	wake=!wake;}	}
	}
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
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
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){}
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);
}

void S800_Beep(int n)
{
	//PK5反转
	GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_5, ~GPIOPinRead(GPIO_PORTK_BASE, GPIO_PIN_5));
	//音频
	if(n==1) SysCtlDelay(ui32SysClock/3000);
	else SysCtlDelay(ui32SysClock/6000);
}
void S800_I2C0_Init(void)
{
	uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
}


uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE)){};
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
  //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(100);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);
	Delay(100);
	return value;
}

/*
	Corresponding to the startup_TM4C129.s vector table systick interrupt program name
*/
void SysTick_Handler(void)
{
	if (systick_1s_couter!= 0)
		systick_1s_couter--;
	else
	{
		systick_1s_couter= SYSTICK_FREQUENCY;
		systick_1s_status = 1;
	}
	
	if (systick_100ms_couter!= 0)
		systick_100ms_couter--;
	else
	{
		systick_100ms_couter= SYSTICK_FREQUENCY/10;
		systick_100ms_status = 1;
	}
	
	if (systick_10ms_couter	!= 0)
		systick_10ms_couter--;
	else
	{
		systick_10ms_couter	= SYSTICK_FREQUENCY/100;
		systick_10ms_status = 1;
		centi_sec++; Time_adjustment();
	}

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

void S800_Timer0_Init(void)
{
	// Enable the Timer0 peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  // Configure Timer0 as a Full-width periodic timer
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  // Set the count time
  TimerLoadSet(TIMER0_BASE, TIMER_A, (ui32SysClock));
  // Enable the timers.
  TimerEnable(TIMER0_BASE, TIMER_A);
}

void TIMER0A_Handler(void)
{
	uint32_t intStatus;
	intStatus = TimerIntStatus(TIMER0_BASE, true);
	TimerIntClear(TIMER0_BASE, intStatus );
	
	if(hour==hour_alarm1 && min==min_alarm1 &&alarm1_ok)		alarm1_ring=true;
	else alarm1_ring=false;
	if(hour==hour_alarm2 && min==min_alarm2 &&alarm2_ok)		alarm2_ring=true;
	else alarm2_ring=false;
}

void Show_student_number(void)
{
	num=0;
	systick_1s_couter= SYSTICK_FREQUENCY;
	systick_1s_status=0;
	while(1)
	{
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
		switch(num)
		{
			case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[2]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
			case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[1]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
		  case 2:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[9]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
			case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[1]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			case 4:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			case 5:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[4]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			case 6:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[1]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			default:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[7]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;		
		}
		if(++num>=8) num=0;
		
		if (systick_1s_status)
		{
			systick_1s_status	= 0;
			break;
		}
	}
}

void Serial_port_function(void)
{
	char a[256];
	if(strncmp(RxBuf,"GET",3)==0 && RxBuf[3]=='+') 
	{
		if(strcmp(RxBuf+4,"DATE")==0) 	
		{
			sprintf(a, "%d-%d-%d",2020,month,day);
			UARTStringPut("Date:");UARTStringPut(a);//回显以当前日期
			UARTStringPut("\r\n");//回车换行
		}
		if(strcmp(RxBuf+4,"TIME")==0)
		{
			if(hour>12&& twelvehours_status) sprintf(a, "pm:%d-%d-%d-%d",hour_process(hour),min,sec,centi_sec);
			else if(hour<=12&& twelvehours_status) sprintf(a, "am:%d-%d-%d-%d",hour,min,sec,centi_sec);
			else sprintf(a, "%d-%d-%d-%d",hour,min,sec,centi_sec);
			UARTStringPut("Time:");UARTStringPut(a);//回显以当前时间
			UARTStringPut("\r\n");//回车换行
		}
	}
	
	if(strncmp(RxBuf,"SET",3)==0 && RxBuf[3]=='+' && RxBuf[13]=='\0') 
	{
		if(strncmp(RxBuf+4,"DATE",4)==0 && RxBuf[8]=='+') 
		{
			if(RxBuf[9]<='9' && RxBuf[9]>='0'&& RxBuf[10]<='9' && RxBuf[10]>='0' && RxBuf[11]<='9' && RxBuf[11]>='0'&& RxBuf[12]<='9' && RxBuf[12]>='0'  ) 
      {	
				month=10*(RxBuf[9]-'0')+RxBuf[10]-'0';
				day=10*(RxBuf[11]-'0')+RxBuf[12]-'0';
				Date_adjustment();
				UARTStringPut("Date_set_already.\r\n");//回显执行成功的语句
			}
		}
		if(strncmp(RxBuf+4,"TIME",4)==0 && RxBuf[8]=='+') 
		{
			if(RxBuf[9]<='9' && RxBuf[9]>='0'&& RxBuf[10]<='9' && RxBuf[10]>='0' && RxBuf[11]<='9' && RxBuf[11]>='0'&& RxBuf[12]<='9' && RxBuf[12]>='0'  ) 
      {	
				hour=10*(RxBuf[9]-'0')+RxBuf[10]-'0';
				min=10*(RxBuf[11]-'0')+RxBuf[12]-'0';
				sec=centi_sec=0;
				Time_adjustment();
				UARTStringPut("Time_set_already.\r\n");//回显执行成功的语句
			}
		}
		if(strncmp(RxBuf+4,"ALM1",4)==0 && RxBuf[8]=='+') 
		{
			if(RxBuf[9]<='9' && RxBuf[9]>='0'&& RxBuf[10]<='9' && RxBuf[10]>='0' && RxBuf[11]<='9' && RxBuf[11]>='0'&& RxBuf[12]<='9' && RxBuf[12]>='0'  ) 
      {	
				hour_alarm1=10*(RxBuf[9]-'0')+RxBuf[10]-'0';
				min_alarm1=10*(RxBuf[11]-'0')+RxBuf[12]-'0';
				Alarm1_adjustment();
				UARTStringPut("Alarm1_set_already.\r\n");//回显执行成功的语句
			}
		}
		if(strncmp(RxBuf+4,"ALM2",4)==0 && RxBuf[8]=='+') 
		{
			if(RxBuf[9]<='9' && RxBuf[9]>='0'&& RxBuf[10]<='9' && RxBuf[10]>='0' && RxBuf[11]<='9' && RxBuf[11]>='0'&& RxBuf[12]<='9' && RxBuf[12]>='0'  ) 
      {	
				hour_alarm2=10*(RxBuf[9]-'0')+RxBuf[10]-'0';
				min_alarm2=10*(RxBuf[11]-'0')+RxBuf[12]-'0';
				Alarm2_adjustment();
				UARTStringPut("Alarm2_set_already.\r\n");//回显执行成功的语句
			}
		}
	}
	if(strcmp(RxBuf,"RESET")==0) //串行口重启
	{
		Show_student_number();
		hour=min=sec=centi_sec=Function=alarm1_ok=alarm2_ok=0;
		month=6;day=20;
		hour_alarm1=7;min_alarm1=0;hour_alarm2=13;min_alarm2=30;
		UARTStringPut("Reset_already.\r\n");//回显执行成功的语句
	}
}

void Time_display(void)
{
	static uint16_t i2c_flash_cnt1=0,i2c_flash_cnt2=0,num=0;
	static uint8_t status1=1,status2=1;
	if(!(wake||alarm1_ring||alarm2_ring)) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);//防拖影
	if(status1 && status2 && (wake||alarm1_ring||alarm2_ring))
	{
		switch(num)
	  {
		  case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour)/10]);
		  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
		  case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour)%10]|0x80);//加上分割号
		  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
		  case 2:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min/10]);
		  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
		  case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min%10]|0x80);
		  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
		  case 4:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[sec/10]);
		  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
		  case 5:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[sec%10]|0x80);
		  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
		  case 6:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[centi_sec/10]);
		  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
		  default:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[centi_sec%10]);
		  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;		
	  }
	  if(++num>=8) num=0;
	}
	if(twelvehours_status && hour>12) pm_status=1; else pm_status=0;
		
	  if (systick_10ms_status&&(wake||alarm1_ring||alarm2_ring))
	  {
		  systick_10ms_status	= 0;
		  if (++i2c_flash_cnt1>= I2C_FLASHTIME/10)
		  {
			  i2c_flash_cnt1	= 0;
				//pm_i2c()函数能指示是否为12小时制的PM
				if(!alarm1_ring && !alarm2_ring)
				{
			  	if (i2c_status &&!alarm1_ok &&!alarm2_ok)	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xfe&pm_i2c());//LED1闪烁
					else  if(i2c_status && alarm1_ok &&!alarm2_ok) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xee&pm_i2c());//LED1、5闪烁
					else  if(i2c_status &&!alarm1_ok && alarm2_ok) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xde&pm_i2c());//LED1、6闪烁
			  	else  if(i2c_status && alarm1_ok && alarm2_ok) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xce&pm_i2c());//LED1、5、6闪烁
					else	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,pm_i2c());
			  	i2c_status	= !i2c_status;
			  }
				if(alarm1_ring) status1=!status1;		else status1=1;//闹钟到时，数码管时间显示1Hz闪烁
		  }
			if (++i2c_flash_cnt2>= I2C_FLASHTIME/20)
		  {
			  i2c_flash_cnt2	= 0;
				if(alarm1_ring) 
				{
					if (i2c_status) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xee&pm_i2c());//LED1、5以2Hz闪烁
					else  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,pm_i2c());	
					i2c_status	= !i2c_status;
				}
				if(alarm2_ring) 
				{
					if (i2c_status) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xde&pm_i2c());//LED1、6以2Hz闪烁
					else  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,pm_i2c());
					status2=!status2;//闹钟到时，数码管时间显示2Hz闪烁
					i2c_status	= !i2c_status;					
				}
			  else 	{	status2=1;	} 
			}
	  }
}

void Date_display(void)
{
	static uint16_t i2c_flash_cnt=0,num=0;
	if(!(wake||alarm1_ring||alarm2_ring))result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	if(wake||alarm1_ring||alarm2_ring)
	{	
		switch(num)
		{
			case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[2]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
			case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
		  case 2:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[2]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
			case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			case 4:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[month/10]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			case 5:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[month%10]|0x80);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			case 6:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[day/10]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			default:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[day%10]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;		
		}
		if(++num>=8) num=0;
	
		if (systick_10ms_status)
		{
			systick_10ms_status	= 0;
			if (++i2c_flash_cnt>= I2C_FLASHTIME/10)
			{
				i2c_flash_cnt	= 0;
				if (i2c_status)	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xfd);//LED2闪烁
				else					  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xff);
				i2c_status	= !i2c_status;
			}
		}
	}
}

void Time_setting(void)
{	
	static uint8_t k=1;//用以数码管闪烁
	static uint16_t i2c_flash_cnt=0,num=0;
	if(!(wake||alarm1_ring||alarm2_ring))result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	if(wake||alarm1_ring||alarm2_ring)
	{	
		if(k){
			switch(num)
			{
				case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour)/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour)%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
		  	case 2:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 4:case 6:case 7:
					result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 5:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			}
			if(++num>=8) num=0;
		}
		else if(set_status){
			switch(num)
			{
		  	case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
				case 2:case 4:case 5:
					result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
			}
			if(++num>=6) num=0;
		}
		else {
			switch(num)
			{
				case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour)/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour)%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 2:case 4:case 5:
					result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
			}
			if(++num>=6) num=0;
		}
		if(twelvehours_status && hour>12) pm_status=1; else pm_status=0;
		
		if (systick_10ms_status)
		{
			systick_10ms_status	= 0;
			if (++i2c_flash_cnt>= I2C_FLASHTIME/10)
			{
				i2c_flash_cnt	= 0;
				if (i2c_status)
					result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xfb&pm_i2c());//LED3闪烁
				else
					result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,pm_i2c());
				i2c_status	= !i2c_status;
				k=!k;  
			}
		}
		
		if(set_status)//调整小时
		{		
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfb) pressed3=true;
			else{
				if(pressed3)	{		pressed3=false;	hour++;Time_adjustment();sec=centi_sec=0;}//调整时将秒数置为零
			}
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xf7) pressed4=true;
			else{
				if(pressed4)	{	pressed4=false;	hour--;Time_adjustment();sec=centi_sec=0;}
			}
		}
		else//调整分钟
		{
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfb) pressed3=true;
			else{
				if(pressed3)	{		pressed3=false;	min++;Time_adjustment();sec=centi_sec=0;}
			}
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xf7) pressed4=true;
			else{
				if(pressed4)	{	pressed4=false;	min--;Time_adjustment();sec=centi_sec=0;}
			}
		}
		
		if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfd) pressed2=true;//切换调整的位置
	  else{
			if(pressed2) {	pressed2=false;	set_status= !set_status;	}
		}
	}
}

void Date_setting(void)
{	
	static uint8_t k=1;//用以数码管闪烁
	static uint16_t i2c_flash_cnt=0,num=0;
	if(!(wake||alarm1_ring||alarm2_ring))result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	if(wake||alarm1_ring||alarm2_ring){
		if(k){
			switch(num)
			{				
				case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[2]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
		  	case 2:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[2]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 4:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[month/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 5:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[month%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 6:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[day/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				default:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[day%10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
			}
			if(++num>=8) num=0;
		}
		else if(set_status){
			switch(num)
			{
				case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[2]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
		  	case 2:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[2]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 4:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[day/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
				default:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[day%10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;	
			}
			if(++num>=6) num=0;
		}
		else {
			switch(num)
			{
				case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[2]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
		  	case 2:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[2]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 4:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[month/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 5:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[month%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			}
			if(++num>=6) num=0;
		}
		
		if (systick_10ms_status)
		{
			systick_10ms_status	= 0;
			if (++i2c_flash_cnt>= I2C_FLASHTIME/10)
			{
				i2c_flash_cnt	= 0;
				if (i2c_status)
					result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xf7);//LED4闪烁
				else
					result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xff);
				i2c_status	= !i2c_status;
				k=!k;
			}
		}
		
		if(set_status)//调整月份
		{		
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfb) pressed3=true;
			else{
				if(pressed3)	{		pressed3=false;	month++;Date_adjustment();	}
			}
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xf7) pressed4=true;
			else{
				if(pressed4)	{	pressed4=false;	month--;Date_adjustment();	}
			}
		}
		else//调整日期
		{
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfb) pressed3=true;
			else{
				if(pressed3)	{		pressed3=false;	day++;Date_adjustment();	}
			}
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xf7) pressed4=true;
			else{
				if(pressed4)	{	pressed4=false;	day--;Date_adjustment();		}
			}
		}
		
		if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfd) pressed2=true;//切换调整的位置
	  else{
			if(pressed2) {	pressed2=false;	set_status	= !set_status;	}
		}
	}
}

void Alarm1_setting(void)
{
	static uint8_t k=1;//用以数码管闪烁
	static uint16_t i2c_flash_cnt=0,num=0;
	if(!(wake||alarm1_ring||alarm2_ring))result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	if(wake||alarm1_ring||alarm2_ring){
		if(k){
			switch(num)
			{
				case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour_alarm1)/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour_alarm1)%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
		  	case 2:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min_alarm1/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min_alarm1%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 4:case 6:case 7:
					result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 5:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			}
			if(++num>=8) num=0;
		}
		else if(set_status){
			switch(num)
			{
		  	case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min_alarm1/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min_alarm1%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
				case 2:case 4:case 5:
					result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
			}
			if(++num>=6) num=0;
		}
		else {
			switch(num)
			{
				case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour_alarm1)/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour_alarm1)%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 2:case 4:case 5:
					result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
			}
			if(++num>=6) num=0;
		}
		if(twelvehours_status && hour_alarm1>12) pm_status=1; else pm_status=0;
    result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xef&pm_i2c());//LED5常亮
		
		if (systick_10ms_status)
		{
			systick_10ms_status	= 0;
			if (++i2c_flash_cnt>= I2C_FLASHTIME/10)	{	i2c_flash_cnt	= 0;	k=!k;  }
		}
		
		if(set_status)//调整小时
		{		
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfb) pressed3=true;
			else{
				if(pressed3)	{		pressed3=false;	hour_alarm1++;Alarm1_adjustment();}
			}
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xf7) pressed4=true;
			else{
				if(pressed4)	{	pressed4=false;	hour_alarm1--;Alarm1_adjustment();}
			}
		}
		else//调整分钟
		{
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfb) pressed3=true;
			else{
				if(pressed3)	{		pressed3=false;	min_alarm1++;Alarm1_adjustment();}
			}
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xf7) pressed4=true;
			else{
				if(pressed4)	{	pressed4=false;	min_alarm1--;Alarm1_adjustment();}
			}
		}
		
		if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfd) pressed2=true;//切换调整的位置
	  else{
			if(pressed2) {	pressed2=false;	set_status= !set_status;	}
		}
	}
}

void Alarm2_setting(void)
{
	static uint8_t k=1;//用以数码管闪烁
	static uint16_t i2c_flash_cnt=0,num=0;
  if(!(wake||alarm1_ring||alarm2_ring)) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xff);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	if(wake||alarm1_ring||alarm2_ring){
		if(k){
			switch(num)
			{
				case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour_alarm2)/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour_alarm2)%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
		  	case 2:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min_alarm2/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min_alarm2%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 4:case 6:case 7:
					result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
				case 5:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;
			}
			if(++num>=8) num=0;
		}
		else if(set_status){
			switch(num)
			{
		  	case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min_alarm2/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min_alarm2%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
				case 2:case 4:case 5:
					result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
			}
			if(++num>=6) num=0;
		}
		else {
			switch(num)
			{
				case 0:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour_alarm2)/10]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 1:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_process(hour_alarm2)%10]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<num);break;	
				case 2:case 4:case 5:
					result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
				  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
				case 3:result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]|0x80);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(num+2));break;
			}
			if(++num>=6) num=0;
		}
	
		if (systick_10ms_status)
		{
			systick_10ms_status	= 0;
			if (++i2c_flash_cnt>= I2C_FLASHTIME/10)		{	i2c_flash_cnt	= 0;k=!k;  	}
		}
		
		if(set_status)//调整小时
		{		
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfb) pressed3=true;
			else{
				if(pressed3)	{		pressed3=false;	hour_alarm2++;Alarm2_adjustment();}
			}
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xf7) pressed4=true;
			else{
				if(pressed4)	{	pressed4=false;	hour_alarm2--;Alarm2_adjustment();}
			}
		}
		else//调整分钟
		{
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfb) pressed3=true;
			else{
				if(pressed3)	{		pressed3=false;	min_alarm2++;Alarm2_adjustment();}
			}
			if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xf7) pressed4=true;
			else{
				if(pressed4)	{	pressed4=false;	min_alarm2--;Alarm2_adjustment();}
			}
		}
		if(twelvehours_status && hour_alarm2>12) pm_status=1; else pm_status=0;
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xdf&pm_i2c());//LED6常亮
		
		if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfd) pressed2=true;//切换调整的位置
	  else{		if(pressed2) {	pressed2=false;	set_status= !set_status;	}	}
	}
}
void Time_adjustment(void)
{
	if(centi_sec>99){centi_sec-=100;sec++;}
	if(sec>=60)	{sec-=60;min++;}
  if(min>=60) {min-=60;hour++;}
	if(hour>=24) hour-=24;
	
	if(centi_sec<0){centi_sec+=100;sec--;}
	if(sec<0)	{sec+=60;min--;}
  if(min<0) {min+=60;hour--;}
	if(hour<0) hour+=24;
}

void Date_adjustment(void)
{
	if(day>monthdays[month]){day-=monthdays[month];month++;}
	if(month>12) month-=12;
	
  if(day<1) {day+=monthdays[month-1];month--;}
	if(month<1) month+=12;
}

void Alarm1_adjustment(void)
{
  if(min_alarm1>=60) {min_alarm1-=60;hour_alarm1++;}
	if(hour_alarm1>=24) hour_alarm1-=24;
	
  if(min_alarm1<0) {min_alarm1+=60;hour_alarm1--;}
	if(hour_alarm1<0) hour_alarm1+=24;
}

void Alarm2_adjustment(void)
{
	if(min_alarm2>=60) {min_alarm2-=60;hour_alarm2++;}
	if(hour_alarm2>=24) hour_alarm2-=24;
	
  if(min_alarm2<0) {min_alarm2+=60;hour_alarm2--;}
	if(hour_alarm2<0) hour_alarm2+=24;
}
