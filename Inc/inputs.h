#ifndef INPUTS_H
#define INPUTS_H


#include "main.h"

#define INPUT_DEBUG 0

typedef struct {
	uint8_t button_1;
	uint8_t button_2;
	uint32_t encoder_1;
	uint32_t encoder_2;
	uint8_t program;

	union
	{
	 uint8_t     changed;      /* Allows us to refer to the flags 'en masse' */
	 struct
	 {
	  uint8_t button1_changed : 1,        /* Explanation of foo */
	          button2_changed : 1,        /* Explanation of bar */
	          encoder1_changed : 1,     /* Unused */
	          encoder2_changed : 1,     /* Unused */
	          program_changed : 1,     /* Unused */
	          spare2 : 1,     /* Unused */
	          spare1 : 1,     /* Unused */
	          spare0 : 1;     /* Unused */
	 };
	};
} input_t;

void ReadInputs(input_t* inputs);
void HandleInputs(input_t* inputs);

// Example usage
//	  ReadInputs(&inputs);
//	  HandleInputs(&inputs);


#endif

