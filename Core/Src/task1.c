/*
 * Task1.c
 *
 *  Created on: May 24, 2024
 *      Author: Daniel Pazzini
 */

#include "task1.h"

#define MAX_VALUE 20
#define MIN_VALUE 1

extern UART_HandleTypeDef huart2;

void pack_formatter (message_t * msg, void * data, queueID_e destine_task, uint32_t data_size);

void StartMytask1(void *argument)
{

	unsigned char txDataBuffer[100] = "Generic info from task1\n\r";

	static uint32_t incrementer_var = MIN_VALUE,
									decrementer_var = MAX_VALUE;
	static message_t msg_sender = {
			.size = 0,
			.id.receiver = NONE_ID,
			.id.sender = NONE_ID,
			.data_ptr = NULL,
	};

  for(;;){

		// Wait till the uart is available
		osMutexAcquire(getMymutexHandle(), osWaitForever);

		// send uart message to user terminal
		HAL_UART_Transmit(&huart2, txDataBuffer, strlen((void *)txDataBuffer), 100);

  	// Release mutex after uart usage
		releaseMymutexHandle();

		// package formatted data and send it to task2
		pack_formatter (&msg_sender, &incrementer_var, TASK2_ID, sizeof(incrementer_var));
		incrementer_var =  (incrementer_var >= MAX_VALUE) ? MIN_VALUE : incrementer_var+1 ;
		osMessageQueuePut(getmyQueueHandle(), &msg_sender, 0, 0);

		// package formatted data and send it to task4
		pack_formatter (&msg_sender, &decrementer_var, TASK4_ID, sizeof(decrementer_var));
		decrementer_var =  (decrementer_var <= MIN_VALUE) ? MAX_VALUE : decrementer_var-1 ;
		osMessageQueuePut(getmyQueueHandle(), &msg_sender, 0, 0);


		osDelay(2000);

  }

}

void pack_formatter (message_t * msg, void * data, queueID_e destine_task, uint32_t data_size){

	msg->data_ptr = data;
	msg->id.sender = TASK1_ID;
	msg->id.receiver = destine_task;
	msg->size = data_size;
}
