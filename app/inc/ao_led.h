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

#ifndef AO_LED_H_
#define AO_LED_H_

/********************** CPP guard ********************************************/
#ifdef __cplusplus
extern "C" {
#endif

/********************** inclusions *******************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
/********************** macros ***********************************************/
#define NUMBER_OF_LEDS 3U
/********************** typedef **********************************************/
typedef enum
{
  AO_LED_MESSAGE_ON,
  AO_LED_MESSAGE_OFF,
  AO_LED_MESSAGE__N,
} ao_led_msg_type_t;

typedef struct 
{
  ao_led_msg_type_t type;
  led_colour_t      colour;
} ao_led_message_t;

typedef struct
{
	GPIO_TypeDef  *port;
	uint16_t 	    pin;
	GPIO_PinState state;
	char 		      colour[10];
} led_info_t;

typedef enum
{
  RED   = 0U,
  GREEN,
  BLUE
} led_colour_t;

typedef struct
{
    QueueHandle_t hqueue;
    TaskHandle_t  htask;
    led_info_t	  info[NUMBER_OF_LEDS]; // use led_t to reference
} ao_led_handle_t;

/********************** external data declaration ****************************/
extern ao_led_handle_t ao_led;

/********************** external functions declaration ***********************/

bool ao_led_send(ao_led_handle_t* hao_led, ao_led_message_t msg);

void ao_leds_init(ao_led_handle_t* hao_led);

/********************** End of CPP guard *************************************/
#ifdef __cplusplus
}
#endif

#endif /* AO_LED_H_ */
/********************** end of file ******************************************/

