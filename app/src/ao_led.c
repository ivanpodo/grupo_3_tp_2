/*
 * Copyright (c) 2023 Sebastian Bedin <sebabedin@gmail.com>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @author : Sebastian Bedin <sebabedin@gmail.com>
 */

/********************** inclusions *******************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "cmsis_os.h"
#include "timers.h"
#include "board.h"
#include "logger.h"
#include "dwt.h"

#include "ao_led.h"

/********************** macros and definitions *******************************/
#define QUEUE_AO_LED_LENGTH_            (5)
#define QUEUE_AO_LED_ITEM_SIZE_         (sizeof(ao_led_message_t))
#define LED_ON_PERIOD_TICKS_			(TickType_t)(500U / portTICK_PERIOD_MS)

#define WAIT_TIME   0U

/********************** internal data declaration ****************************/

ao_led_handle_t ao_led =
				{
					.info[RED].port 	= LD3_GPIO_Port,
					.info[RED].pin 	= LD3_Pin,
					.info[RED].state = GPIO_PIN_RESET,
					.info[RED].colour = "RED",

					.info[GREEN].port 	= LD1_GPIO_Port,
					.info[GREEN].pin 	= LD1_Pin,
					.info[GREEN].state = GPIO_PIN_RESET,
					.info[GREEN].colour = "GREEN",
					
					.info[BLUE].port 	= LD2_GPIO_Port,
					.info[BLUE].pin 	= LD2_Pin,
					.info[BLUE].state = GPIO_PIN_RESET,
					.info[BLUE].colour = "BLUE",
				};

/********************** internal functions declaration ***********************/
static void ao_led_init_(ao_led_handle_t* hao_led);
static void ao_led_finish_(ao_led_handle_t* hao_led);

/********************** internal data definition *****************************/
static bool ao_running = false;

/********************** external data definition *****************************/

/********************** internal functions definition ************************/

static void ao_task_(void *argument)
{
  ao_led_handle_t *hao = (ao_led_handle_t *)argument;

  LOGGER_INFO("AO LED started")

  while (true)
  {
    ao_led_message_t msg;

    LOGGER_INFO("AO LED \t- Waiting event");

	/* hacemos? bool ao_running = true */
    ao_running = true;

    while (pdPASS == xQueueReceive(hao->hqueue, &msg, (TickType_t)0U))
    {
		switch (msg.type)
		{
		case AO_LED_MESSAGE_ON:
			HAL_GPIO_WritePin(hao->info[msg.colour].port, hao->info[msg.colour].pin, GPIO_PIN_SET);
			vTaskDelay(LED_ON_PERIOD_TICKS_);
			HAL_GPIO_WritePin(hao->info[msg.colour].port, hao->info[msg.colour].pin, GPIO_PIN_RESET);
			break;

		/*Intentional fallthrough*/
		case AO_LED_MESSAGE_OFF:
		case AO_LED_MESSAGE__N:
		
		default:
			LOGGER_INFO("AO LED - bad ao message");
			break;
		}
    }

	LOGGER_INFO("AO LED - releasing memory");

	ao_led_finish_(hao);
	/* hacemos? bool ao_running = false */
	
	/*
	 * librar queue
	 * asignar null al ptr
	 * */
//	vQueueDelete(hao->hqueue);
//	hao->hqueue = NULL;
//
//	hao->htask = NULL;
//	ao_running = false;
//	vTaskDelete(NULL);

	/* No se ejecuta*/
  }
}

/********************** external functions definition ************************/

bool ao_led_send(ao_led_handle_t* hao_led, ao_led_message_t msg)
{
	/* (verificar si )el ao esta corriendo
	 * crear queue, reasignar puntero
	 * enviar evento a la cola
	 * crear tarea, reasignar puntero
	 * */
	if(false == ao_running)
	{
		ao_led_init_(hao_led);
	}

	return (pdPASS == xQueueSend(hao_led->hqueue, (void*)&msg, (TickType_t)0U));
}

/*
 * Hacemos init o no hace falta? porque si vamos a crear y destruir queue y task,
 * no es requerido. Podemos utilizarlo en 'ao_led_send()
 */

static void ao_led_init_(ao_led_handle_t* hao_led)
{
  // Queues
  hao_led->hqueue = xQueueCreate(QUEUE_AO_LED_LENGTH_, QUEUE_AO_LED_ITEM_SIZE_);
  configASSERT(NULL != hao_led->hqueue);

  // Tasks
  BaseType_t status;
  status = xTaskCreate
		  (
			  ao_task_,
			  "task_ao_led",
			  128,
			  (void* const)hao_led,
			  (tskIDLE_PRIORITY + 1),
			  &hao_led->htask
		  );

  configASSERT(pdPASS == status);

  ao_running = true;
}

/*
 * This function will be excecuted up to vTaskDelete.
 */
static void ao_led_finish_(ao_led_handle_t* hao_led)
{
	vQueueDelete(hao_led->hqueue);
	ao_running = false;
	vTaskDelete(hao_led->htask);
	// this won't be executed!
}

/********************** end of file ******************************************/
