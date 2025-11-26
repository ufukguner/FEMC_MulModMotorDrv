/*
 * Parameter.h
 *
 *
 *      Author: UG
 */

#ifndef SRC_PARAMETER_H_
#define SRC_PARAMETER_H_

#include "stm32f4xx_hal.h"
#include <math.h>

#define DRV8962_EN1_PORT      GPIOB
#define DRV8962_EN1_PIN       GPIO_PIN_6

#define DRV8962_EN2_PORT      GPIOB
#define DRV8962_EN2_PIN       GPIO_PIN_7

#define DRV8962_EN3_PORT      GPIOB
#define DRV8962_EN3_PIN       GPIO_PIN_8

#define DRV8962_EN4_PORT      GPIOB
#define DRV8962_EN4_PIN       GPIO_PIN_9

#define DRV8962_SLEEP_PORT    GPIOB
#define DRV8962_SLEEP_PIN     GPIO_PIN_0

#define DRV8962_FAULT_PORT    GPIOB
#define DRV8962_FAULT_PIN     GPIO_PIN_2   // Fault feedback

#define DRV8962_MODE_PORT     GPIOB
#define DRV8962_MODE_PIN      GPIO_PIN_3




#define DC_PWM_TIMER          htim2
#define DC_PWM_CH1            TIM_CHANNEL_1   // IN1
#define DC_PWM_CH2            TIM_CHANNEL_2   // IN2


#define STEP_PWM_TIMER        htim2
#define STEP_PWM_CH1          TIM_CHANNEL_1   // IN1
#define STEP_PWM_CH2          TIM_CHANNEL_2   // IN2
#define STEP_PWM_CH3          TIM_CHANNEL_3   // IN3
#define STEP_PWM_CH4          TIM_CHANNEL_4   // IN4


#define BLDC_PWM_TIMER        htim2
#define BLDC_PWM_CH_U         TIM_CHANNEL_1   // Phase U
#define BLDC_PWM_CH_V         TIM_CHANNEL_2   // Phase V
#define BLDC_PWM_CH_W         TIM_CHANNEL_3   // Phase W


#define ENC1_TIMER            htim1   // Quadrature encoder A/B1
#define ENC2_TIMER            htim8   // Quadrature encoder A/B2

#define MAGENC_SPI_HANDLE     hspi2   // AS5048 için SPI2
#define MAGENC_CS_PORT        GPIOB
#define MAGENC_CS_PIN         GPIO_PIN_12

#define TWO_PI  (2.0f * M_PI)
#define HALF_PI  ((float)M_PI_2)

#define MSG_RATE         2500   // micro second

#define STX         0x02
#define ETX         0x03


void DRV8962_Sleep_ON();
void DRV8962_Sleep_OFF();
uint8_t  DRV8962_Read_FAULT();
void DRV8962_Enable_ALL();
void DRV8962_Enable_Pin(uint32_t pin);
void DRV8962_Disable_ALL();
void DRV8962_Enable_ALL();
float clampf(float val, float min, float max);
float angle_diff(float a, float b);



#endif /* SRC_PARAMETER_H_ */
