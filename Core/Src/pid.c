/*
 * pid.c
 *
 *  Created on: 16 mar 2021
 *      Author: antoniopellegrini
 */

#include "pid.h"
#include "math.h"


int PID_Calculate(hpid * hpid, float elapsed_time ,float current_angle, float desired_angle, float kp, float ki, float kd, float multiplier){



	//hpid passed by REFERENCE
	hpid->prev_e = hpid->e;
	hpid->e = desired_angle - current_angle;
	hpid->P = kp * hpid->e;
	hpid->I_old = hpid->I_prev;
	hpid->I_prev = hpid->I_prev + (ki * hpid->e);


	if ( fabs(hpid->calculated_pid) >= 4000.0  ){ // anti windup
		hpid->I_prev = hpid->I_old;
		}else{
			hpid->I_prev = 0;
		}


	hpid->D = kd * ((hpid->e - hpid->prev_e)/elapsed_time);

	//calculate pid and direction (rigth - left)

	hpid->calculated_pid = (hpid->P + hpid->I_prev + hpid->D) * multiplier;

	//limit to 4000
	if ( fabs(hpid->calculated_pid) > 4000.0  ){
		hpid->calculated_pid = (hpid->calculated_pid >= 0) ? 4000 : - 4000 ;
	}

	hpid->calculated_direction = ( hpid->calculated_pid >= 0 ) ? 1 : 0;


	return 0;
}


float PID_Get_Actuation(hpid * hpid){
	return hpid->calculated_pid;
}
int PID_Get_Direction(hpid * hpid){
	return hpid->calculated_direction;
}


