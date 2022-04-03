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


void Trigger1_Handler(void)
{
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
	
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, PWM_Load_Val / 2);  // 100% duty cycle PA6
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, PWM_Load_Val);  // 100% duty cycle PA7
	
	// Set the Duty Cycle for PD0 & PD1 
	
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWM_Load_Val);  // 100% duty cycle PD0
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWM_Load_Val);  // 100% duty cycle PD1
	
	// PWM enable PA6 and PA7
	
	PWMOutputState( PWM1_BASE, PWM_OUT_6_BIT, true); 
	PWMOutputState( PWM1_BASE, PWM_OUT_7_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);	
	
	
	// PWM Enable PD0 & PD1 
	
	PWMOutputState( PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMOutputState( PWM0_BASE, PWM_OUT_1_BIT, true);
	PWMGenEnable( PWM0_BASE, PWM_GEN_3);
	
	
}

void PWM_FORWARD(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
	//GPIO_PORTA_DEN_R |= 0x04; // PA2; IN1 
	//GPIO_PORTA_DEN_R |= 0x08; // PA3; IN2 
	
	//GPIO_PORTA_DIR_R |= 0x04; // PA2
	//GPIO_PORTA_DIR_R |= 0x08; // PA3
	
	// FRONT-LEFT WHEEL FOWARD
	
	GPIO_PORTA_DATA_R &= ~0x04;  // PA2 IN1
	GPIO_PORTA_DATA_R |= 0x08; // PA3 IN2
	
	
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








void PWM_LEFT(void)
{
}


void PWM_RIGHT(void)
{
}















int main(void)
{
	
	PWM_INIT();
	PWM_FORWARD();
	
	
	
	while(1)
	{
		
		
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
}
