#ifndef LCD_H
#define LCD_H

#include "main.h"

void LCD_command(uint16_t i);
void LCD_write(uint16_t i);
void LCD_init();



// Example usage
//  LCD_init();
//
//  char h[] = "Hello World";
//  uint16_t len = strlen(h);
//  for (uint16_t i = 0; i < len; i++) {
//	  LCD_write(h[i]);
//  }


#endif
