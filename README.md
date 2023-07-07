# DieukhientocdodongcothongquaRF
PWM
/*
 * main.c

 *
 *  Created on: 15/06/2023
 *      Author: Tran Duc Nhan
 */
// PWM Example

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include "myLCD.h"

// Output Port pin LED_O
#define PORT_LED_O         PORTB
#define DDR_LED_O          DDRB
#define BIT_LED_O          6

// Output Port pin MOTO1
#define PORT_MOTOR1        PORTE
#define DDR_MOTOR1         DDRE
#define MOTOR1 			   4
#define MOTOR2 			   5

// Define frequency
#define PWM_MAX_DUTY_CYCLE 0x3FF

void PWM_vInit(void)
{
    /*
       Start Timer 1 with clock prescaler CLK/8 and phase correct
       10-bit PWM mode. Output on PB6 (OC1B). Resolution is 1.09 us.
       Frequency is 450 Hz.
    */
    TCCR3A =  (0<<COM3A1)|(0<<COM3A0)|(0<<COM3B1)|(0<<COM3B0)
             |(0<<COM3C1)|(0<<COM3C0)|(0<<WGM31) |(0<<WGM30);

    TCCR3B =  (0<<ICNC3) |(0<<ICES3) |(0<<WGM33) |(0<<WGM32)
             |(0<<CS32)  |(0<<CS31)  |(1<<CS30);

    // Reset counter
    TCNT3 = 0;
 
    // Set duty cycle to 0%
    OCR3A = 0;
}

void PWM_vSetDutyCycle(uint16_t u16DutyCycle)
{
    // Clip parameter to maximum value
    if (u16DutyCycle > PWM_MAX_DUTY_CYCLE)
    {
        u16DutyCycle = PWM_MAX_DUTY_CYCLE;
    }

    OCR3A = u16DutyCycle;
}

int main(void)
{

	int incrflag = 0;
	//init_LCD();
	//move_LCD(1, 1);
	//printf_LCD("LED");
    uint16_t ledIntensity = 0;
    // Set LED_O as output pin
    DDR_LED_O |= (1<<BIT_LED_O);
    // Set MOTOS as output pin
    DDR_MOTOR1 |= (1<<MOTOR1)|(1<<MOTOR2);
    // Initialise PWM
    PWM_vInit();

    // Set duty cycle to 1%
    PWM_vSetDutyCycle(PWM_MAX_DUTY_CYCLE/100);
    // Repeat indefinitely
    PWM_vSetDutyCycle(1024);
    for(;;)
    {
    	PORT_MOTOR1 &= ~((1<<MOTOR1)|(1<<MOTOR2));
    	PORT_MOTOR1 |= (1<<MOTOR2);
    	_delay_ms(5000);
    	PORT_MOTOR1 &= ~((1<<MOTOR1)|(1<<MOTOR2));
    	PORT_MOTOR1 |= (1<<MOTOR1);
    	_delay_ms(5000);
    }

}
