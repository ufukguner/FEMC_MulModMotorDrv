/*
 * BLDC.h
 *
 *
 *      Author: UG
 */

#ifndef SRC_BLDC_H_
#define SRC_BLDC_H_

#pragma once
#include "stm32f4xx_hal.h"
#include "Encoder.h"
#include <math.h>
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
float PolePair;
float enc_offset;   // mekanik ofset (rad)
int   enc_dir;      // +1 veya -1 (mekanik yön)
} BLDC_Par_t;

void BLDC_Motor_Con();
void BLDC_Init(MCCommand *RecvCMD);
void BLDC_InitPosition(void);

#endif /* SRC_BLDC_H_ */
