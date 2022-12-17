/*
 * PID.h
 *
 * Created: 22/11/2021 10:59:19
 *  Author: Henning
 */ 


#ifndef PID_H_
#define PID_H_

#define PID_KP 1
#define PID_KI 0.01
#define PID_KD 0
#define OCR1A_MAX 185
#define OCR1A_MIN 20

float PID_calculate(float RPM_current, float RPM_setpoint, float RPM_time);

#endif /* PID_H_ */