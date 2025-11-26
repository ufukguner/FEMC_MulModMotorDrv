/*
 * CombineDrv.h
 *
 *
 *      Author: mcpne
 */

#ifndef SRC_MOTORIDENT_H_
#define SRC_MOTORIDENT_H_
#include "stm32f4xx_hal.h"
#include "ProcessCmd.h"

typedef enum {
    CH1 = TIM_CHANNEL_1,
    CH2 = TIM_CHANNEL_2,
    CH3 = TIM_CHANNEL_3,
    CH4 = TIM_CHANNEL_4
} TIM_CH_t;
typedef struct {
	uint8_t Mot_Type; // Motor type
	uint32_t Active_Ch[4]; // Active channel info
} ComDrv_Par_t;
void TIM2_SetMode_Center(void);
void TIM2_SetMode_Up(void);
int Check_Motor_Type_Channel(MCCommand *RecvCMD);

#endif /* SRC_MOTORIDENT_H_ */
