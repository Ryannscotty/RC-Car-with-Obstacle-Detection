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


#define PWM_FREQ 50











void PORT_F_A_C_Init(void)
{
	uint32_t ui32_Loop;
	
	
	// Set the Clock for PORTF and PORTA 
	
	SYSCTL_RCGC2_R |= 0x20; // PORTF Clock Set 
	SYSCTL_RCGC2_R |= 0x01; // PORTA Clock set
	SYSCTL_RCGC2_R |= 0x04; // PORTC Clock Set
	// Dummy Read for Few Clock Cycles
	
	ui32_Loop = SYSCTL_RCGC2_R;
	
	// Set GPIO PINS PF1(Red LED), PF2(BLUE LED), PF3(GREEN LED) to Outputs
	
	GPIO_PORTF_DIR_R |= 0x0E;
	
	// Enable PORtF Pins For Digital Use
	
	GPIO_PORTF_DEN_R |= 0x0E; 
	GPIO_PORTF_AMSEL_R &= ~ 0x0E; // Disable analog functions on PORTF
		
}


 


void UART0_Init(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);
	// 20 MHZ clock frequncy 
	// Activate Clock of UART0

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}


void UART1_Init(void)
{
	
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);
	// 20 MHZ clock frequncy 
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	GPIOPinConfigure(GPIO_PC4_U1RX);
	GPIOPinConfigure(GPIO_PC5_U1TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	
	
	IntDisable(INT_UART1);
	IntMasterEnable();
	IntEnable(INT_UART1);
	UARTIntEnable(UART1_BASE, UART_INT_RX);
	
	
}




void PORT_FUNCT_INIT(void)
{
	// Front Wheel PINS 
	
	// PORT A  PA6(ENA), PA7(ENB), PA2(IN1), PA3(IN2), PA4(IN3) and PA5(IN4) as Digital pins 
	
	// FRONT-LEFT WHEEL INIT
	
	GPIO_PORTA_DEN_R |= 0x40; // PA6; M1PWM2
	GPIO_PORTA_DEN_R |= 0x04; // PA2; IN1 
	GPIO_PORTA_DEN_R |= 0x08; // PA3; IN2 
	
	GPIO_PORTA_DIR_R |= 0x40; // PA6
	GPIO_PORTA_DIR_R |= 0x04; // PA2
	GPIO_PORTA_DIR_R |= 0x08; // PA3
	
	// FRONT-RIGHT WHEEL INIT
	
	GPIO_PORTA_DEN_R |= 0x80; // PA7; M1PWM3
	GPIO_PORTA_DEN_R |= 0x10; // PA4; IN3
	GPIO_PORTA_DEN_R |= 0x20; // PA5; IN4
	
	GPIO_PORTA_DIR_R |= 0x80; // PA7
	GPIO_PORTA_DIR_R |= 0x10; // PA4
	GPIO_PORTA_DIR_R |= 0x20; // PA5
	
	// REAR-LEFT WHELL INIT
	
	GPIO_PORTD_DEN_R |= 0x01;  // PD0; M0PWM6
	GPIO_PORTD_DEN_R |= 0x04;  // PD2; IN1
	GPIO_PORTD_DEN_R |= 0x08;  // PD3; IN2
	
	GPIO_PORTD_DIR_R |= 0x01;  // PD0
	GPIO_PORTD_DIR_R |= 0x08;  // PD3
	GPIO_PORTD_DIR_R |= 0x04;  // PD2
	
	// REAR-RIGHT WHEEL INIT
	
	GPIO_PORTD_DEN_R |= 0x02;  // PD1; M0PWM7
	GPIO_PORTD_DEN_R |= 0x40;  // PD6; IN3
	GPIO_PORTD_DEN_R |= 0x80;  // PD7; IN4
	//GPIO_PORTF_DEN_R |= 0x10;   // PF4
	
	GPIO_PORTD_DIR_R |= 0x02; // PD1
	GPIO_PORTD_DIR_R |= 0x40; // PD6
	GPIO_PORTD_DIR_R |= 0x80; // PD7
	//GPIO_PORTF_DIR_R |= 0x10;  // PF4
	
	
}

void PWM_INIT(void)
{
	// set the clock rate for the CPU at 20 MHz Freq
	
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);
	
	// define the load value 
	
	volatile uint32_t PWM_Clock;
	volatile uint32_t PWM_Load_Val;
	
	//  Start the clock for PORT A and M1PWM2 & M1PWM3
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	
	// Start the Clock for PORT D and M0PWM6 & M0PWM7
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	
	
	// Add PORT FUNCtion INIT
	
	PORT_FUNCT_INIT();
	
	// set the PWM Clock Divider to 32 
	
	SysCtlPWMClockSet(SYSCTL_PWMDIV_32);
	
	// Config PINs as PWM pins PA6 & PA7
	
	GPIOPinConfigure(GPIO_PA6_M1PWM2);
	GPIOPinConfigure(GPIO_PA7_M1PWM3);
	
	// Config PINs as PWM pins PD0 & PD1
	
	GPIOPinConfigure(GPIO_PD0_M0PWM6);
	GPIOPinConfigure(GPIO_PD1_M0PWM7);
	
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	
	// configure load value for the duty cycle for motor speed
	
	PWM_Clock = SysCtlClockGet() / 32;  //  PWM_Clock = 20Mhz / 32 = 625 khz clock freq
	
	PWM_Load_Val = (PWM_Clock / PWM_FREQ) - 1; // load_value = 12500 Hz
	
	// Configure PWM For PA6 and PA7 
	
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN); 
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWM_Load_Val);
	
	// Configure PWM for PD0 & PD1 
	
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWM_Load_Val);
	
	
	
	// set the duty cycle for PA6 and PA7 
	
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, PWM_Load_Val / 2);  // 50% duty cycle PA6
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, PWM_Load_Val / 2);  // 50% duty cycle PA7
	
	// Set the Duty Cycle for PD0 & PD1 
	
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWM_Load_Val / 2);  // 50% duty cycle PD0
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWM_Load_Val / 2);  // 50% duty cycle PD1
	
	// PWM enable PA6 and PA7
	
	PWMOutputState( PWM1_BASE, PWM_OUT_6_BIT, true); 
	PWMOutputState( PWM1_BASE, PWM_OUT_7_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);	
	
	
	// PWM Enable PD0 & PD1 
	
	PWMOutputState( PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMOutputState( PWM0_BASE, PWM_OUT_1_BIT, true);
	PWMGenEnable( PWM0_BASE, PWM_GEN_3);
	
	
}

// SENSOR CONTROLS



void Milli_Second_Delay( int time)
{
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);
	
	
	
	unsigned long Period = 20000;
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
	
	for( i=0; i<time; i++)  // creating the delay in milli-seconds 
	{
		while((TIMER2_RIS_R & 0x01)==0)
		{
			TIMER2_ICR_R |= 0x01;
		}
		
	}
	
}

void TrigSend(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN); // 20 mhz
	
	
	// start the clock for Port B
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	
		
	// set PB6 as digital pin
	
	GPIO_PORTB_DEN_R |= 0x40;
	
	// set PB6 as digital output pin
	
	GPIO_PORTB_DIR_R |= 0x40;
	
	
	GPIO_PORTB_DATA_R |= 0x40;
	SysCtlDelay( SysCtlClockGet() / (100000 * 3));
	//Micro_Second_Delay(10);
	GPIO_PORTB_DATA_R &= ~0x40;
	
	
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
	
	
	
	while(1)
	{
		Milli_Second_Delay(60);
		TrigSend();
		Milli_Second_Delay(60);
			
	}
	
}


void Echo1_Init(void)
{
	// Start the clock for PORT B 
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN); // 20 mhz
	SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER3);
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
	TimerControlEvent( TIMER3_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);
	
	
	// TIMER 3A Interrupt Configure
	
	IntEnable(INT_TIMER3A);
	IntPrioritySet(INT_TIMER3A, 0x00);
	TimerIntEnable(TIMER3_BASE,TIMER_CAPA_EVENT);
	IntMasterEnable();
	
	
	 
	 // Enable Timer 3A 
	 
	 TimerEnable( TIMER3_BASE, TIMER_A);
	 
	
}


void STOP(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
	// FRONT-LEFT WHEEL STOP
	
	GPIO_PORTA_DATA_R &= ~0x04;  // PA2 IN1
	GPIO_PORTA_DATA_R &= ~0x08; // PA3 IN2
	
	
	// FRONT-RIGHT WHEEL STOP
	
	GPIO_PORTA_DATA_R &= ~0x10; // PA4 IN3
	GPIO_PORTA_DATA_R &= ~0x20;  // PA5 IN4
	
	
	
	// REAR-LEFT WHEEl STOP
	
	GPIO_PORTD_DATA_R &= ~0x04;  // PD2 IN1
	GPIO_PORTD_DATA_R &= ~0x08; // PD3 IN2
	
	
	// REAR-RIGHT WHEEL STOP 
	
	GPIO_PORTD_DATA_R &= ~0x40; // PD6 IN3
	GPIO_PORTD_DATA_R &= ~0x80;  // PD7 IN4  PF4
}








uint32_t Sensor_Distance_Handler(void)
{
	uint32_t ui32Sensor_status;
	ui32Sensor_status= TimerIntStatus(TIMER3_BASE, true);
	TimerIntClear(TIMER3_BASE, ui32Sensor_status);
	
	
	
	int Rising_edge;
	int Falling_edge;
	volatile uint32_t Pulse_duration;
	 unsigned long distance = 0;
	
	TIMER3_ICR_R = 0x04;
	
	if((TIMER3_RIS_R & 0x04)==0)
	{
		Rising_edge = TIMER3_TAR_R;
	}
	
	TIMER3_ICR_R = 0x04;
	
	if((TIMER3_RIS_R & 0x04)==1)
	{
		Falling_edge = TIMER3_TAR_R;
		
	}
	
	Pulse_duration = ( Falling_edge - Rising_edge );
	
	 distance = (( Pulse_duration * 10625) / 10000000) & 0x00FFFFFF;
	
	
	char Dis[20];
		int i =0;
	int DT;
	DT = SysCtlClockGet();
	
	
	
		i =0;
		//distance = ( Pulse_duration * 10625) / 10000000;
	sprintf( Dis, "\r\nObsticle= %d cm ",distance);
		
	while( Dis[i] != '\0')
		{
			UARTCharPut(UART0_BASE, Dis[i]);
			i++;
			SysCtlDelay(DT/ (3 * 3));
			
		}
		
		
	if( distance <= 50)
	{
		STOP();
	}
	
}





void PWM_FORWARD(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
	
	// FRONT-LEFT WHEEL FOWARD
	
	GPIO_PORTA_DATA_R |= 0x04;  // PA2 IN1
	GPIO_PORTA_DATA_R &= ~0x08; // PA3 IN2
	
	
	// FRONT-RIGHT WHEEL FORWARD
	
	GPIO_PORTA_DATA_R &= ~0x10; // PA4 IN3
	GPIO_PORTA_DATA_R |= 0x20;  // PA5 IN4
	
	
	
	// REAR-LEFT WHEEl FORWARD
	
	GPIO_PORTD_DATA_R &=~0x04;  // PD2 IN1
	GPIO_PORTD_DATA_R |= 0x08; // PD3 IN2
	
	
	// REAR-RIGHT WHEEL FORWARD 
	
	GPIO_PORTD_DATA_R |= 0x40; // PD6 IN3
	GPIO_PORTD_DATA_R &= ~0x80;  // PD7 IN4  PF4
}




void PWM_BACKWARD(void)
{
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
	// FRONT-LEFT WHEEL RIGHT
	
	GPIO_PORTA_DATA_R &= ~0x04;  // PA2 IN1
	GPIO_PORTA_DATA_R |= 0x08; // PA3 IN2
	
	
	// FRONT-RIGHT WHEEL RIGHT
	
	GPIO_PORTA_DATA_R |= 0x10; // PA4 IN3
	GPIO_PORTA_DATA_R &= ~0x20;  // PA5 IN4
	
	
	
	// REAR-LEFT WHEEl RIGHT
	
	GPIO_PORTD_DATA_R |= 0x04;  // PD2 IN1
	GPIO_PORTD_DATA_R &= ~0x08; // PD3 IN2
	
	
	// REAR-RIGHT WHEEL RIGHT
	
	GPIO_PORTD_DATA_R |= 0x40; // PD6 IN3
	GPIO_PORTD_DATA_R &= ~0x80;  // PD7 IN4  PF4
	
}











void PWM_RIGHT(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
	
	// FRONT-LEFT WHEEL RIGHT
	
	GPIO_PORTA_DATA_R &= ~0x04;  // PA2 IN1
	GPIO_PORTA_DATA_R |= 0x08; // PA3 IN2
	
	
	// FRONT-RIGHT WHEEL RIGHT
	
	GPIO_PORTA_DATA_R |= 0x10; // PA4 IN3
	GPIO_PORTA_DATA_R &= ~0x20;  // PA5 IN4
	
	
	
	// REAR-LEFT WHEEl RIGHT
	
	GPIO_PORTD_DATA_R |= 0x04;  // PD2 IN1
	GPIO_PORTD_DATA_R &= ~0x08; // PD3 IN2
	
	
	// REAR-RIGHT WHEEL RIGHT
	
	GPIO_PORTD_DATA_R |= 0x40; // PD6 IN3
	GPIO_PORTD_DATA_R &= ~0x80;  // PD7 IN4  PF4
}


void PWM_LEFT(void)
{
	
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
	// FRONT-LEFT WHEEL BACKWARD
	
	GPIO_PORTA_DATA_R &=~0x04;  // PA2 IN1
	GPIO_PORTA_DATA_R |= 0x08; // PA3 IN2
	
	// FRONT-RIGHT WHEEL BACKWARD
	
	GPIO_PORTA_DATA_R |= 0x10;  // PA4
	GPIO_PORTA_DATA_R &= ~0x20; // PA5
	
	// REAR-LEFT WHEEL BACKWARD
	
	GPIO_PORTD_DATA_R &= ~0x04;  // PD2 IN1
	GPIO_PORTD_DATA_R |= 0x08; // PD3 IN2
	
	// REAR-RIGHT WHEEL BACKWARD
	
	GPIO_PORTD_DATA_R |= 0x40; // PD6 IN3
	GPIO_PORTD_DATA_R &= ~0x80;  // PD7 IN4
	
}

void UART1_Handler(void)
{
	uint32_t ui32RX_status;
	ui32RX_status= UARTIntStatus(UART1_BASE, true);
	UARTIntClear(UART1_BASE, ui32RX_status);
	char b;
	char d;
	int delay;
	delay= SysCtlClockGet();
PWM_INIT();
	
	while(1)
	{
	
	while(UARTCharsAvail(UART1_BASE))
	{
		
		
			b=UARTCharGet(UART1_BASE);
			UARTCharPut(UART1_BASE,b);
		int delay2 = SysCtlClockGet();
		
		switch(b)
		{
			case 'F' :
				
			PWM_FORWARD();
				GPIO_PORTF_DATA_R |= 0x02;
			SysCtlDelay((delay2 * 2) / 3 );
			STOP();
				GPIO_PORTF_DATA_R &=~0x02;
			break;
			
			case 'B' :
				
			PWM_BACKWARD();
				GPIO_PORTF_DATA_R |= 0x04;
				SysCtlDelay((delay2 * 2) / 3 );
			STOP();
			GPIO_PORTF_DATA_R &= ~0x04;
			break;
			
			case 'L' :
				
			PWM_LEFT();
				GPIO_PORTF_DATA_R |= 0x08;
			SysCtlDelay((delay2 * 2) / 3 );
			STOP();
				GPIO_PORTF_DATA_R &= ~0x08;
			break;
			
			case 'R' :
				PWM_RIGHT();
				GPIO_PORTF_DATA_R |= 0x08;
			SysCtlDelay((delay2 * 2) / 3 );
			STOP();
				GPIO_PORTF_DATA_R &= ~0x08;
			break;
			
			default:  
				
				STOP(); 
			
		}
		
		
		
		
		
		
		
		
		
		
		/*
		
			if(b =='K')
			{
				while( b=='K')
				{
				PWM_FORWARD();
				GPIO_PORTF_DATA_R |= 0x02;
				}
			}
			if(b=='k')
			{
			while( b=='k')
			{
				STOP();
				GPIO_PORTF_DATA_R &=~0x02;
			}
	   	} 
			if(b=='B')
			{
				PWM_BACKWARD();
				GPIO_PORTF_DATA_R |= 0x04;
			}
			
			else
			{
				STOP();
				GPIO_PORTF_DATA_R &= ~0x04;
			}
			
			if(b=='L')
			{
				PWM_LEFT();
				GPIO_PORTF_DATA_R |= 0x08;
			}
			
			else
			{
				STOP();
				GPIO_PORTF_DATA_R &= ~0x08;
			}
			
			if(b== 'R')
			{
				PWM_RIGHT();
				GPIO_PORTF_DATA_R |= 0x08;
				
			}
			
			else 
			{
				STOP();
				GPIO_PORTF_DATA_R &= ~0x08;
			}
			*/
			
		}
	
	}
		
		
		
	}
	
	

















int main(void)
{
	
 PORT_F_A_C_Init();
 UART0_Init();
 UART1_Init();
 PWM_INIT();	
 Trigger1_Init();
 Echo1_Init();	
	
	
	
	
	
	volatile uint32_t distance;
	
	distance = Sensor_Distance_Handler();
	
	char Dis[20];
		int i =0;
	int DT;
	DT = SysCtlClockGet();
	
	while(1)
	{
		i =0;
		//distance = Sensor_Distance1();
		
		sprintf( Dis, "\r\nObsticle= %d cm ",distance);
		
		while( Dis[i] != '\0')
		{
			UARTCharPut(UART0_BASE, Dis[i]);
			i++;
			SysCtlDelay(DT/ (10 * 3));
			
		}
	}
	
	
	
}










