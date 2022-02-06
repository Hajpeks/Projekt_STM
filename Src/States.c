/*
 * States.c
 *
 *  Created on: Feb 3, 2022
 *      Author: hpiet
 */

#include "States.h"
#include "main.h"
#include "tim.h"

double Temp_Diff;
static unsigned short heater_state,fan_state;

/**
 *
 * @brief States_Change is a main function in our program,
 * its job is to measure difference between TempRef and temp,
 * then react with heater(if difference is positive and bigger than 0.04 degree),
 * or with fan(if difference is negative and less than -0.10).
 *
 * @param TempRef is our set temperature received from a user via UART
 * @param temp is our current temperature
 *
 */

void State_Change (double TR, double t) ////dodatkowo temp zadana bedzie potem
{
	Temp_Diff=TR-t;
	if(Temp_Diff > 0.04){
		heater_state=1;
		fan_state=0;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	}
	else if(Temp_Diff<-0.10){
		heater_state=0;
		fan_state=1;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
	}
	else
	{
		heater_state=0;
		fan_state=0;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	}

}

/**
 *
 * @brief States_Run function in our program turns on GPIO Ports,
 * which our external control mechanism is connected to STM.
 * It's launching each port according to present state of pin,
 * depending on current states provided by States_Change function.
 *
 * @param None
 *
 */

void States_Run(){
	HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, heater_state);  	// Heater control
	HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, fan_state);          //Fan_Control
}
