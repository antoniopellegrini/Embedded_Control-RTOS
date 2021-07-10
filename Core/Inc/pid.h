/*
 * pid.h
 *
 *  Created on: 16 mar 2021
 *      Author: antoniopellegrini
 */

#ifndef INC_PID_H_
#define INC_PID_H_


//private variables




typedef struct hpid{
	float I_prev, I_old, prev_e, e;
	float pid_elapsedTime, pid_currentTime, pid_previousTime;
	float P,I,D;
	float calculated_pid;
	int calculated_direction;
}hpid;


//functions prototypes

int PID_Calculate(hpid * hpid, float elapsed_time ,float current_angle, float desired_angle, float kp, float ki, float kd, float multiplier);
float PID_Get_Actuation(hpid * hpid);
int PID_Get_Direction(hpid * hpid);




#endif /* INC_PID_H_ */
