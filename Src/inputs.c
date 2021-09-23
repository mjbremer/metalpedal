/*
 * inputs.c
 *
 *  Created on: Sep 23, 2021
 *      Author: mjb2
 */
#include "inputs.h"

extern TIM_HandleTypeDef htim1;

void ReadInputs(input_t* inputs) {

	input_t new_inputs;
	memcpy(&new_inputs,inputs,sizeof(input_t));

	new_inputs.button_1 = HAL_GPIO_ReadPin(USERBUTTON1_GPIO_Port, USERBUTTON1_Pin);
	new_inputs.button_2 = HAL_GPIO_ReadPin(USERBUTTON2_GPIO_Port, USERBUTTON2_Pin);
	new_inputs.encoder_1 = htim1.Instance->CNT;

	if (new_inputs.button_1 != inputs->button_1)
		new_inputs.button1_changed = 1;
	if (new_inputs.button_2 != inputs->button_2)
		new_inputs.button2_changed = 1;
	if (new_inputs.encoder_1 != inputs->encoder_1)
			new_inputs.encoder1_changed = 1;



	memcpy(inputs,&new_inputs,sizeof(input_t));
}


void HandleInputs(input_t* inputs) {

#if INPUT_DEBUG
	  char debug_msg[100];
	  int debug_msg_len;
	  if (inputs->changed) {
		  if (inputs->button1_changed) {
			 debug_msg_len = snprintf(debug_msg, 100, "Button 1 changed to %d\r\n", inputs.button_1);
			 HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, debug_msg_len, 10000000U);
		  }

		  if (inputs->button2_changed) {
			 debug_msg_len = snprintf(debug_msg, 100, "Button 2 changed to %d\r\n", inputs.button_2);
			 HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, debug_msg_len, 10000000U);
		  }
		  if (inputs->encoder1_changed) {
			 debug_msg_len = snprintf(debug_msg, 100, "Encoder 1 changed to %lu\r\n", inputs.encoder_1);
			 HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, debug_msg_len, 10000000U);
		  }

	  }
#endif

		  inputs->changed = 0;

}
