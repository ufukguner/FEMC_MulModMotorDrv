/*
 * StepMotor.h
 *
 *
 *      Author: UG
 */

#ifndef SRC_STEPMOTOR_H_
#define SRC_STEPMOTOR_H_

#pragma once
#include "stm32f4xx_hal.h"
#include "math.h"
#include "ProcessCmd.h"


typedef struct {
int encoder;
uint8_t Con_type; // position or speed
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
float PreErr;
float Gain;
float Step_Speed; // hız [rev/s]
uint32_t  Microstep;
int   enc_dir;
} Step_Par_t;

void StepperLUT_Init(MCCommand *RecvCMD);
void Stepper_Con();

#endif /* SRC_STEPMOTOR_H_ */
