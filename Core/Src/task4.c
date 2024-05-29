/*
 * task4.c
 *
 *  Created on: May 29, 2024
 *      Author: Daniel Pazzini
 */

#include "task4.h"

bool isMessageForTask4(message_t msg);

extern UART_HandleTypeDef huart2;

void StartMytask4(void *argument)
{

	static message_t received_message = {
			.id = {0},
			.size = 0,
			.data_ptr = NULL,
	};

	uint8_t local_string [100] = " ";

  for(;;)
  {

  	osMessageQueueGet(getmyQueueHandle(), &received_message, NULL , osWaitForever);

		// Assuring I'm getting the information from the correct sender
		if(isMessageForTask4(received_message)) {

			// Wait till the uart access gets free
			osMutexAcquire(getMymutexHandle(), osWaitForever);

			snprintf((void *)local_string,sizeof(local_string), "%d - Sent from task %u - to task 4 \n\r", *((int *) received_message.data_ptr) , received_message.id.sender );
			HAL_UART_Transmit(&huart2, local_string, strlen((void *)local_string), 100);

			releaseMymutexHandle();

		} else {

			osMessageQueuePut(getmyQueueHandle(), &received_message, 0, 0);

		}

    osDelay(1);
  }

}

bool isMessageForTask4(message_t msg){
	return (msg.id.receiver & TASK4_ID);
}
