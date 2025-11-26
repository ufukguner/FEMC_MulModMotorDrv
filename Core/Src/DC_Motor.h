/*
 * DC_Motor.h
 *
 *
 *      Author: UG
 */

#ifndef SRC_DC_MOTOR_H_
#define SRC_DC_MOTOR_H_

#pragma once
#include "stm32f4xx_hal.h"
#include "adc_dma.h"
#include "Encoder.h"
#include <stdint.h>
#include "ProcessCmd.h"

typedef struct {
int encoder;
uint32_t Con_frq;
uint32_t Sample;
uint32_t Sample_Cnt;
int Enable;
float rate;
float Gear;
float Vel;
float Pos;
float Kp;
float Ki;
float Kd;
float Ref_Vel;
float Ref_Pos;
float Integral;
float I_Integral;
float I_Kp;
float I_Ki;
float Imax;
float PreErr;
int   enc_dir;
} DC_Par_t;

void DC_Motor_Con();
void DC_Init(MCCommand *RecvCMD);




#endif /* SRC_DC_MOTOR_H_ */
