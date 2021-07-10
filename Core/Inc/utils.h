/*
 * utils.h
 *
 *  Created on: 16 mar 2021
 *      Author: antoniopellegrini
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

typedef struct Process_Timer{

	float current_time;
	float old_time;
	float elapsed_time_in_seconds;
	float mission_time_in_seconds;
	float mission_starting_time;
	float mission_end_time;

} Process_Timer;


void init_process_timer(Process_Timer * process_timer);
void calculate_next_process_timings( Process_Timer * process_timer);

//functions protorypes

int UTIL_Constrain(int num, int min , int max);



#endif /* INC_UTILS_H_ */
