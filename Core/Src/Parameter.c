/*
 * Parameter.c
 *
 *
 *      Author: UG
 */
#include "Parameter.h"



uint8_t  DRV8962_Read_FAULT(){
	return HAL_GPIO_ReadPin(DRV8962_FAULT_PORT, DRV8962_FAULT_PIN) ;
}
void DRV8962_Sleep_ON() {
	HAL_GPIO_WritePin(DRV8962_SLEEP_PORT, DRV8962_SLEEP_PIN, GPIO_PIN_RESET);
}

void DRV8962_Sleep_OFF(){
	HAL_GPIO_WritePin(DRV8962_SLEEP_PORT, DRV8962_SLEEP_PIN, GPIO_PIN_SET);
}

void DRV8962_Enable_ALL(){
        HAL_GPIO_WritePin(DRV8962_EN1_PORT, DRV8962_EN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DRV8962_EN2_PORT, DRV8962_EN2_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DRV8962_EN3_PORT, DRV8962_EN3_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DRV8962_EN4_PORT, DRV8962_EN4_PIN, GPIO_PIN_SET);
 }

void DRV8962_Disable_ALL(){

        HAL_GPIO_WritePin(DRV8962_EN1_PORT, DRV8962_EN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DRV8962_EN2_PORT, DRV8962_EN2_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DRV8962_EN3_PORT, DRV8962_EN3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DRV8962_EN4_PORT, DRV8962_EN4_PIN, GPIO_PIN_RESET);
}
void DRV8962_Enable_Pin(uint32_t pin){
	switch (pin) {
		case TIM_CHANNEL_1:
			HAL_GPIO_WritePin(DRV8962_EN1_PORT, DRV8962_EN1_PIN, GPIO_PIN_SET);
			break;
		case TIM_CHANNEL_2:
			HAL_GPIO_WritePin(DRV8962_EN2_PORT, DRV8962_EN2_PIN, GPIO_PIN_SET);
			break;
		case TIM_CHANNEL_3:
			HAL_GPIO_WritePin(DRV8962_EN3_PORT, DRV8962_EN3_PIN, GPIO_PIN_SET);
			break;
		case TIM_CHANNEL_4:
			HAL_GPIO_WritePin(DRV8962_EN4_PORT, DRV8962_EN4_PIN, GPIO_PIN_SET);
			break;

	}
}

float clampf(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

//Angle difference  (+pi,-pi)
float angle_diff(float a, float b){
    float d = a - b;

    if (d >  M_PI)
    	d -= TWO_PI;
    if (d < -M_PI)
    	d += TWO_PI;

    return d;
}
