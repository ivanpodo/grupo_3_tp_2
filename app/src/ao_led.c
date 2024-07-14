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
#define LED_ON_PERIOD_MS_				(TickType_t)(1000U / portTICK_PERIOD_MS)

/********************** internal data declaration ****************************/

ao_led_handle_t ao_led_red =
				{
					.info.port 	= LD3_GPIO_Port,
					.info.pin 	= LD3_Pin,
					.info.state = GPIO_PIN_RESET,
					.info.colour = "RED"
				};

ao_led_handle_t ao_led_green =
				{
					.info.port 	= LD1_GPIO_Port,
					.info.pin 	= LD1_Pin,
					.info.state = GPIO_PIN_RESET,
					.info.colour = "GREEN"
				};

ao_led_handle_t ao_led_blue =
				{
					.info.port 	= LD2_GPIO_Port,
					.info.pin 	= LD2_Pin,
					.info.state = GPIO_PIN_RESET,
					.info.colour = "BLUE"
				};

/********************** internal functions declaration ***********************/
static void ao_led_timer_cb_(TimerHandle_t pxTimer);

/********************** internal data definition *****************************/

/********************** external data definition *****************************/

/********************** internal functions definition ************************/

static void ao_task_(void *argument)
{
  ao_led_handle_t *hao = (ao_led_handle_t *)argument;

  LOGGER_INFO("AO LED %s started", hao->info.colour)

  while (true)
  {
    ao_led_message_t msg;

    LOGGER_INFO("LED %s\t- Waiting event", hao->info.colour);

    if (pdPASS == xQueueReceive(hao->hqueue, &msg, portMAX_DELAY))
    {
      GPIO_PinState led_state;

      switch(msg)
      {
      case AO_LED_MESSAGE_ON:
          led_state = GPIO_PIN_SET;
          xTimerStart(hao->htimer, (TickType_t) 0U);
          LOGGER_INFO("LED %s\t- Turn ON", hao->info.colour);
    	  break;

      case AO_LED_MESSAGE_OFF:
          led_state = GPIO_PIN_RESET;
          LOGGER_INFO("LED %s\t- Turn OFF", hao->info.colour);
    	  break;

      default:
    	  LOGGER_INFO("LED %s\t- ERROR - bad led message", hao->info.colour);
    	  break;
      }

      HAL_GPIO_WritePin(hao->info.port, hao->info.pin, led_state);
    }
  }
}

/********************** external functions definition ************************/

bool ao_led_send(ao_led_handle_t* hao_led, ao_led_message_t msg)
{
  return (pdPASS == xQueueSend(hao_led->hqueue, (void*)&msg, (TickType_t)0U));
}

void ao_leds_init(ao_led_handle_t* hao_led_red, ao_led_handle_t* hao_led_green, ao_led_handle_t* hao_led_blue)
{
  // Queues
  hao_led_red->hqueue   = xQueueCreate(QUEUE_AO_LED_LENGTH_, QUEUE_AO_LED_ITEM_SIZE_);
  hao_led_green->hqueue = xQueueCreate(QUEUE_AO_LED_LENGTH_, QUEUE_AO_LED_ITEM_SIZE_);
  hao_led_blue->hqueue  = xQueueCreate(QUEUE_AO_LED_LENGTH_, QUEUE_AO_LED_ITEM_SIZE_);
  configASSERT(NULL != hao_led_red->hqueue);
  configASSERT(NULL != hao_led_green->hqueue);
  configASSERT(NULL != hao_led_blue->hqueue);

  // Timers
  hao_led_red->htimer   = xTimerCreate
		  	  	  	  	  (
							  "timer_ao_led_red",
							  LED_ON_PERIOD_MS_,
							  false,
							  hao_led_red,
							  ao_led_timer_cb_
		  	  	  	  	  );
  configASSERT(NULL != hao_led_red->htimer);

  hao_led_green->htimer = xTimerCreate
						  (
							  "timer_ao_led_green",
							  LED_ON_PERIOD_MS_,
							  false,
							  hao_led_green,
							  ao_led_timer_cb_
						  );
  configASSERT(NULL != hao_led_green->htimer);

  hao_led_blue->htimer  = xTimerCreate
						  (
							  "timer_ao_led_blue",
							  LED_ON_PERIOD_MS_,
							  false,
							  hao_led_blue,
							  ao_led_timer_cb_
						  );
  configASSERT(NULL != hao_led_blue->htimer);

  // Tasks
  BaseType_t status;
  status = xTaskCreate
		  (
			  ao_task_,
			  "task_ao_led_red",
			  128,
			  (void* const)hao_led_red,
			  (tskIDLE_PRIORITY + 1),
			  &hao_led_red->htask
		  );
  configASSERT(pdPASS == status);

  status = xTaskCreate
		  (
			  ao_task_,
			  "task_ao_led_green",
			  128,
			  (void* const)hao_led_green,
			  (tskIDLE_PRIORITY + 1),
			  &hao_led_green->htask
		  );
  configASSERT(pdPASS == status);

  status = xTaskCreate
		  (
			  ao_task_,
			  "task_ao_led_blue",
			  128,
			  (void* const)hao_led_blue,
			  (tskIDLE_PRIORITY + 1),
			  &hao_led_blue->htask
	      );
  configASSERT(pdPASS == status);
}

static void ao_led_timer_cb_(TimerHandle_t pxTimer)
{
	ao_led_handle_t *hao_led = (ao_led_handle_t *)pvTimerGetTimerID(pxTimer);
	ao_led_message_t evt = AO_LED_MESSAGE_OFF;
	LOGGER_INFO("LED %s\t- Timer callback executed!", hao_led->info.colour);

	xQueueSendFromISR(hao_led->hqueue, &evt, pdFALSE);
}

// We assume all tasks have the same priority level. If not, we should use
// something like this:
//
// BaseType_t xHigherPriorityTaskWoken = pdTRUE;
// xQueueSendFromISR( ... , ... , &xHigherPriorityTaskWoken);
// portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // Switch context if
// argument is pdTRUE;

/********************** end of file ******************************************/
