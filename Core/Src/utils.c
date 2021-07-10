/*
 * utils.c
 *
 *  Created on: 16 mar 2021
 *      Author: antoniopellegrini
 */


#include "utils.h"





int UTIL_Constrain(int num, int min , int max)
{

	const int temp = num < min ? min : num;
	return temp > max ? max : num;

}

void init_process_timer(Process_Timer * pt)
{
	pt->mission_starting_time = xTaskGetTickCount();
}

void calculate_next_process_timings(Process_Timer * pt)
{
	pt->current_time = xTaskGetTickCount() - pt->mission_starting_time;
	pt->elapsed_time_in_seconds = pt->current_time - pt->old_time;
	pt->mission_time_in_seconds += pt->elapsed_time_in_seconds;

	pt->old_time = pt->current_time;
}

