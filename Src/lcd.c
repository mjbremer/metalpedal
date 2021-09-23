/*
 * lcd.c
 *
 *  Created on: Sep 23, 2021
 *      Author: mjb2
 */

#include "lcd.h"

void LCD_command(uint16_t i)
{
 //P1 = i; //put data on output Port

 GPIOD->BSRR = i | ((~i)<<16);

 //D_I =0; //D/I=LOW : send instruction

 HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);

 //R_W =0; //R/W=LOW : Write

 HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);

 //E = 1;

 HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
 HAL_Delay(100); //enable pulse width >= 300ns
 //E = 0; //Clock enable: falling edge

 HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
}
/**********************************************************/
void LCD_write(uint16_t i)
{
	 //P1 = i; //put data on output Port

	 GPIOD->BSRR = (i&0xff) | (((~i)&0xff)<<16);

	 //D_I =0; //D/I=LOW : send instruction

	 HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);

	 //R_W =0; //R/W=LOW : Write

	 HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);

	 //E = 1;

	 HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	 HAL_Delay(100); //enable pulse width >= 300ns
	 //E = 0; //Clock enable: falling edge

	 HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
}
/**********************************************************/
void LCD_init()
{

HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
 HAL_Delay(100); //Wait >40 msec after power is applied
 LCD_command(0x30); //command 0x30 = Wake up
 HAL_Delay(30); //must wait 5ms, busy flag not available
 LCD_command(0x30); //command 0x30 = Wake up #2
 HAL_Delay(10); //must wait 160us, busy flag not available
 LCD_command(0x30); //command 0x30 = Wake up #3
 HAL_Delay(10); //must wait 160us, busy flag not available
 LCD_command(0x38); //Function set: 8-bit/2-line
 LCD_command(0x10); //Set cursor
 LCD_command(0x0f); //Display ON; Cursor ON
 LCD_command(0x06); //Entry mode set


}
