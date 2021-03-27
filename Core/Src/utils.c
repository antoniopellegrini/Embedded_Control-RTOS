/*
 * utils.c
 *
 *  Created on: 16 mar 2021
 *      Author: antoniopellegrini
 */


#include "utils.h"





int UTIL_Constrain(int num, int min , int max){

	const int temp = num < min ? min : num;
	return temp > max ? max : num;

}



