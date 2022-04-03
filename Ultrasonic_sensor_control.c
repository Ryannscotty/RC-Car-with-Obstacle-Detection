#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_types.h"
//#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"




void UART0_Init(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);
	// 20 MHZ clock frequncy 
	// Activate Clock of UART0

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	
	// PORT F PINS
	
	GPIO_PORTF_DEN_R |= 0x04;
	GPIO_PORTF_DIR_R |= 0x04;
	
	
	
	
	
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}


void Trigger1_Init(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN); // 20 mhz
	
	
	// start the clock for Port B
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	
	// set PB6 as digital pin
	
	GPIO_PORTB_DEN_R |= 0x40;
	
	// set PB6 as digital output pin
	
	GPIO_PORTB_DIR_R |= 0x40;
	
}


void Trigger1_Delay(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);
	
	// use TIMER 1A for Periodic TIMER IN DOWN COUNT for 60 Mircoseconds 
	unsigned long Period1 = 120000;
	
	SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB);
	
	// start the clock for Timer 1A 
	
	SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER1);
	
	// disable Timer 1A
	
	TimerDisable( TIMER1_BASE, TIMER_A);
	
	// Configure Timer 1A In periodic, down-count mode, @ 16-bit
	
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet( TIMER1_BASE, TIMER_A, Period1-1);
	
	
	// interrupt every 60 microseconds
	
	IntEnable(INT_TIMER1A);
	IntPrioritySet(INT_TIMER1A, 0x00);
	TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	
	
	
	// Enable Timer 1A 
	
	TimerEnable( TIMER1_BASE, TIMER_A);
	
}


















void Micro_Second_Delay( int time)
{
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);
	
	
	
	unsigned long Period = 20;
	// start the clock for PORT B
	
	SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB);
	
	// start the clock for Timer 2A 
	
	SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER2);
	
	// disable Timer 2A
	
	TimerDisable( TIMER2_BASE, TIMER_A);
	
	// Configure Timer 2A In periodic, down-count mode, @ 16-bit
	
	TimerConfigure(TIMER2_BASE, TIMER_CFG_A_PERIODIC);
	TimerLoadSet( TIMER2_BASE, TIMER_A, Period-1);
	
	// Enable Timer 2A 
	
	TimerEnable( TIMER2_BASE, TIMER_A);
	
	
	int i;
	
	for( i=0; i<time; i++)  // creating the delay in microseconds 
	{
		while((TIMER2_RIS_R & 0x01)==0)
		{
			TIMER2_ICR_R |= 0x01;
		}
		
	}
	
}


void Trigger1_Handler(void)
{
	
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);
	
	
	
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	
	GPIO_PORTB_DATA_R |= 0x40;
	SysCtlDelay( SysCtlClockGet() / (100000 * 3));
	//Micro_Second_Delay(10);
	GPIO_PORTB_DATA_R &= ~0x40;
	
	
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	
	// PORT F PINS
	
	//GPIO_PORTF_DEN_R |= 0x04;
	//GPIO_PORTF_DIR_R |= 0x04;
	
	
	
	//GPIO_PORTF_DATA_R &= ~0x04;
	//SysCtlDelay( SysCtlClockGet() / 3);
	//GPIO_PORTF_DATA_R |= 0x04;
	
	
	
	
	
}














void Echo1_Init(void)
{
	// Start the clock for PORT B 
	
	SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB);
	
	// Digitally Enable PB2
	
	GPIO_PORTB_DEN_R |= 0x04;
	
	// set PB2 as Digital Input
	
	GPIO_PORTB_DIR_R &= ~0x04;
	
	// set PB1 for Alt. Function
	
	//GPIO_PORTB_AFSEL_R |=0x04;
	
	//GPIO_PORTB_PCTL_R &= ~0x000000F0;
	
	GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2);
	
	
	// Start the Clock For Timer 3A
	
	SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER3);
	
	
	
	// Disable Timer 3A 
	
	TimerDisable( TIMER3_BASE, TIMER_A);
	
	// configure timer 3A in capture mode. edge-time
	
	TimerConfigure( TIMER3_BASE, TIMER_CFG_A_CAP_TIME_UP);
	TimerControlEvent( TIMER3_BASE, TIMER_A, TIMER_EVENT_POS_EDGE| TIMER_EVENT_NEG_EDGE);
	
	
	// TIMER 3A Interrupt Configure
	
	//IntEnable(INT_TIMER3A);
	//IntPrioritySet(INT_TIMER3A, 0x00);
	//TimerIntEnable(TIMER3_BASE,TIMER_CAPA_EVENT);
	//IntMasterEnable();
	
	
	 
	 // Enable Timer 3A 
	 
	 TimerEnable( TIMER3_BASE, TIMER_A);
	 
	
}


/*void Sensor_Distance_Handler(void)
{
	TimerIntClear(TIMER3_BASE, TIMER_CAPA_EVENT);
	int Rising_edge;
	int Falling_edge;
	double Pulse_duration;
	volatile uint32_t distance;
	
	if((GPIO_PORTB_DATA_R & 0x04)==0)
	{
		Rising_edge = TIMER3_TAR_R;
	}
	else if((GPIO_PORTB_DATA_R & 0x04)==1)
	{
		Falling_edge = TIMER3_TAR_R;
		
	}
	Pulse_duration = ( Falling_edge - Rising_edge );
	
	//distance = ( Pulse_duration * 10625) / 10000000;
	
	
	char Dis[20];
		int i =0;
	int DT;
	DT = SysCtlClockGet();
	
	
	
		i =0;
		distance = ( Pulse_duration * 10625) / 10000000;
	sprintf( Dis, "\r\nObsticle= %d cm ",distance);
		
		while( Dis[i] != '\0')
		{
			UARTCharPut(UART0_BASE, Dis[i]);
			i++;
			SysCtlDelay(DT/ (3 * 3));
			
		}
	
	
	
	
	
	
	
	
	
}

*/












 uint32_t Sensor_Distance1(void)
{
	
	
	int Rising_Edge;
		int Falling_Edge;
		unsigned long Pulse_Duration;
		 double  Distance1;
	
	
	//while(1)
	//{
	
	// Trigger1 pulse send
	/*
	GPIO_PORTB_DATA_R &= ~0x40;
	Micro_Second_Delay(10);
	GPIO_PORTB_DATA_R |= 0x40;
	Micro_Second_Delay(10);
	GPIO_PORTB_DATA_R &= ~0x40;
	//GPIO_PORTB_DATA_R &= ~0x40;
	Micro_Second_Delay(60);
	GPIO_PORTB_DATA_R |= 0x40;
	Micro_Second_Delay(60);
	GPIO_PORTB_DATA_R &= ~0x40;
	*/
	// detect the rising and falling edges of echo pin
	
	
	
	
	
		// clear the flag bit ( set the bit)
		TIMER3_ICR_R=0x04;
		
		// capture the rising edge
		while((GPIO_PORTB_DATA_R & 0x04)==0)
		{
			Rising_Edge = TIMER3_TAR_R;  // assign the rising edge to capture time stamp
		 
			// clear the flag bit ( set the bit)
			//TIMER3_ICR_R =4;
   	}
		
		
		// capture the Falling edge
		if((GPIO_PORTB_DATA_R & 0x04)==1)
			{
				TIMER3_ICR_R =0x04;
			Falling_Edge = TIMER3_TAR_R; // assign the Falling edge to capture time stamp
			}
		
		
		Pulse_Duration = ( Falling_Edge - Rising_Edge );  // time difference 
		
		Distance1 = ( Pulse_Duration * 10625) / 10000000;   // the distance in centimeters
			//Distance1 = ( Pulse_Duration * 34300) / 2;	
		return Distance1;
		
		}
	//}
	
		
	
	








int main(void)
{
	UART0_Init();
	Trigger1_Init();
	Trigger1_Delay();
	Echo1_Init();
	
	
	
	int DT;
	DT = SysCtlClockGet();

	
	
	volatile uint32_t distance;
	
	distance = Sensor_Distance1();
	
	char Dis[20];
		int i =0;
	
	while(1)
	{
		//char Dis[20];
	  i =0;
		distance = Sensor_Distance1();
		
		sprintf( Dis, "\r\nObsticle= %d cm ",distance);
		
		while( Dis[i] != '\0')
		{
			UARTCharPut(UART0_BASE, Dis[i]);
			i++;
			SysCtlDelay(DT/ (10 * 3));
			
		}
		
		if( distance <= 100)
		{
			GPIO_PORTF_DATA_R |= 0x04;
		}
		
	}
	
	
	
}







