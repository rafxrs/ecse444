/*
 * cmax.c
 *
 *  Created on: Sep 15, 2025
 *      Author: rafae
 */
#include "main.h"
void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex) {
	(*max) = array[0];
	(*maxIndex) = 0;
	for (uint32_t i = 1; i < size; i++) {
		if (array[i] > (*max)) {
			(*max) = array[i];
			(*maxIndex) = i;
		} // if
	} // for
} // cMax
