/*
 * utils.c
 *
 *  Created on: 16 mar 2021
 *      Author: antoniopellegrini
 */


#include "utils.h"


/**
  * @brief  Constrain a number between two values
  * @param  num Input number
  * @param  min Minumum number
  * @param  max Maximum number
  * @retval constrained Input number
  */
int UTIL_Constrain(int num, int min , int max){

	const int temp = num < min ? min : num;
	return temp > max ? max : num;

}



