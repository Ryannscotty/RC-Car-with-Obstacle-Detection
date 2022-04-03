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


void Sensor_Distance_Handler(void)
	
{
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

	
	while(1)
	{
	while(UARTCharsAvail(UART1_BASE))
	{
		
		
			b=UARTCharGet(UART1_BASE);
			UARTCharPut(UART1_BASE,b);
		
			if(b== 'F')
			{
				GPIO_PORTF_DATA_R |= 0x02;
				
			}
			else
			{
				GPIO_PORTF_DATA_R &=~0x02;
			}
			if(b=='B')
			{
				GPIO_PORTF_DATA_R |= 0x04;
			}
			else
			{
				GPIO_PORTF_DATA_R &= ~0x04;
			}
			if(b=='L')
			{
				GPIO_PORTF_DATA_R |= 0x08;
			}
			else
			{
				GPIO_PORTF_DATA_R &=~0x08;
			}
		
		
			
	}
	
}
	
	
	
	

	
	
	
	
}






int main(void)
{
	
	 PORT_F_A_C_Init();
	
	UART0_Init();
	UART1_Init();
	
	
	
	
	
	
	
	
	
	while(1)
	{
		
	}
	
	
	
}













































































/*//Globally enable interrupts 
void IntGlobalEnable(void)
{
    __asm("    cpsie   i\n");
}


void UART0_Init(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);
	// 20 MHZ clock frequncy 
	// Activate Clock of UART0
	
	SYSCTL_RCGCUART_R |= 0x01; 
	
	// Enable PORTA RX PA0 and TX PA1 for Alt Funtion
	
	GPIO_PORTA_AFSEL_R |= 0x03;
	
	// PORTA control Function 
	
	GPIO_PORTA_PCTL_R |= 0x00000011; 
	 GPIO_PORTA_DEN_R |= 0x03; // Digital Enable PORTA PINS 
	GPIO_PORTA_AMSEL_R &= ~0x03; // disable analog funtion
	
	// Disable the UARTO Module
	
	UART0_CTL_R &= ~0x01;
	
	// Baud Rate Generator 
	
	// (20,000,000 / ( 16* 115,200) ) = 10.85069
	// (0.85069 *64) = 54
	
	UART0_IBRD_R = 10;
	UART0_FBRD_R = 54;
	
	// write the serial parameters 
	
	UART0_LCRH_R = 0x060;
	
	// selecting the system clock 
	
	UART0_CC_R = 0x00; 
	
	// enable UART0 , RX , TX
	
	UART0_CTL_R |= 0x301;
	
}






void UART1_Init(void)
{
	
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);
	// 20 MHZ clock frequncy 
	// Activate Clock of UART1
	
	
	
	// set the clock for UART 1 module 
	
	SYSCTL_RCGCUART_R |= 0x02;
	
	
	// select PC4 and PC5 for ALt. Func pin set
	
	GPIO_PORTC_AFSEL_R |= 0x30; 
	
	// select the PORT Control for GPIO PORT C PC4 and PC5 pins 
	
	GPIO_PORTC_PCTL_R |= 0x00110000; 
	GPIO_PORTC_DEN_R |= 0x30;
	GPIO_PORTC_AMSEL_R &= ~0x30;
	
	// // Disable the UARTO Module
	
	UART1_CTL_R &= ~0x01;
	
	// Baud Rate Generator 
	
	// (20,000,000 / ( 16* 115,200) ) = 10.85069
	// (0.85069 *64) = 54
	
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 38400,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	
	
	
	
	//UART1_IBRD_R = 10;
	//UART1_FBRD_R = 54;
	
	// write the serial parameters 
	
	//UART1_LCRH_R = 0x060;
	
	// selecting the system clock 
	
	//UART1_CC_R = 0x00; 
	
	// enable UART0 , RX , TX
	
	UART1_CTL_R |= 0x301;
	
	// UART1 Interrupt Setup 
	
	IntMasterEnable();
	IntEnable(INT_UART1);
	UARTIntEnable(UART1_BASE, UART_INT_RX);
	
	
	
	
		
}

void UART1_Handler(void)
{
	
	uint32_t ui32RX_status;
	ui32RX_status= UARTIntStatus(UART1_BASE, true);
	UARTIntClear(UART1_BASE, ui32RX_status);
	char BT;
	char BT_R;
	
}


int main(void)
{
	
	
	PORT_F_A_C_Init();
	
	
	UART0_Init();
	UART1_Init();
	
	
	
	while(1)
	{
}

}
*/
