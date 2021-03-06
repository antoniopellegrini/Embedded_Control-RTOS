/*
 * pid.c
 *
 *  Created on: 16 mar 2021
 *      Author: antoniopellegrini
 */

#include "pid.h"

/**
  * @brief   Calculates 	pid actuation variable
  * @param   hpid  			Pid handler
  * @param   current_time   Current time in ticks
  * @param   current_angle  Current angle (input)
  *	@param   desired_angle  Desired angle (output)
  *	@params  kp,ki,kd 		Pid gains
  *	@retrival calculated_pid, calculated_direction are saved in hpid
  */
int PID_Calculate(hpid * hpid, float current_time ,float current_angle, float desired_angle, float kp, float ki, float kd){

	//hpid passed by REFERENCE
	hpid->prev_e = hpid->e;
	hpid->pid_currentTime = current_time;
	hpid->pid_elapsedTime = (hpid->pid_currentTime - hpid->pid_previousTime) / 1000; // Divide by 1000 to get seconds
	hpid->pid_previousTime = hpid->pid_currentTime;

	hpid->e = desired_angle - current_angle;
	hpid->P = kp * hpid->e;
	hpid->I_prev = hpid->I_prev + (ki * hpid->e);
	hpid->D = kd * ((hpid->e - (hpid->prev_e))/(hpid->pid_elapsedTime));

	//calculate pid and dirextion (rigth - left)
	hpid->calculated_pid = hpid->P + hpid->I_prev - hpid->D;
	hpid->calculated_direction = (hpid->calculated_pid >= 0) ? 1 : 0;

	return 0;
}


float PID_Get_Actuation(hpid * hpid){
	return hpid->calculated_pid;
}
int PID_Get_Direction(hpid * hpid){
	return hpid->calculated_direction;
}





// hpid passed by VALUE
//	hpid.prev_e = hpid.e;
//	hpid.pid_currentTime = current_time;
//	hpid.pid_elapsedTime = (hpid.pid_currentTime - hpid.pid_previousTime) / 1000; // Divide by 1000 to get seconds
//	hpid.pid_previousTime = hpid.pid_currentTime;
//
//	hpid.e = desired_angle * current_angle;
//	hpid.P = kp * hpid.e;
//	hpid.I_prev = hpid.I_prev + (ki * hpid.e);
//	hpid.D = kd * ((hpid.e - (hpid.prev_e))/(hpid.pid_elapsedTime));
//
//	//calculate pid and dirextion (rigth - left)
//	hpid.calculated_pid = hpid.P + hpid.I_prev - hpid.D;
//	hpid.calculated_direction = (hpid.calculated_pid >= 0) ? 1 : 0;
