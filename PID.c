/*
 * PID.c
 *
 * Created: 22/11/2021 10:58:57
 *  Author: Henning
 */ 

#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "i2cmaster.h"
#include "usart.h"
#include "PID.h"
float PID_calculate(float ADC_current, float ADC_setpoint, float ADC_time)
{
	static float ADC_lastinput=0;
	static double I_error=0;
	
	float P_error=ADC_setpoint-ADC_current;
	
	I_error+=P_error*ADC_time;
	
	float D_error=(ADC_current-ADC_lastinput)/ADC_time;
	ADC_lastinput=ADC_current;
	return (PID_KP*P_error+PID_KI*I_error+PID_KD*D_error);
}