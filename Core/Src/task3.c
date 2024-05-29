/*
 * task3.c
 *
 *  Created on: May 27, 2024
 *      Author: Daniel Pazzini
 */

#include <stdio.h>
#include "task3.h"

extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;

void StartMytask3(void *argument)
{

	uint8_t adc_buffered_value [10] = " ";
	static message_t msg_sender = {
			.size = sizeof(" - From task3 to task2\n\r") - 1,
			.id.receiver = TASK2_ID,
			.id.sender = TASK3_ID,
			.data_ptr = " - From task3 to task2\n\r"
	};
	float adc_value = 0.0f;

  while(1){

  	// Regardless the task priority, it release the adc update just after the correct external interruption
  	osSemaphoreAcquire(getNewDataSemaphore(), osWaitForever);

  	// Wait till the uart is available
  	osMutexAcquire(getMymutexHandle(), osWaitForever);

  	adc_value = HAL_ADC_GetValue(&hadc1);
  	snprintf((void *)adc_buffered_value, sizeof(adc_buffered_value), "%.2f", adc_value);
  	HAL_UART_Transmit(&huart2, (void *)&adc_buffered_value, strlen((void *)adc_buffered_value), 100);

  	// Release mutex after uart usage
  	releaseMymutexHandle();

  	// Redundancy to avoid a failure during a packet sent
  	for (uint32_t try_again = 0; try_again < 3 ; try_again++){
			if (osMessageQueuePut(getmyQueueHandle(), &msg_sender, 0, 0) == osOK){
				break;
			}
  	}

  }
}
