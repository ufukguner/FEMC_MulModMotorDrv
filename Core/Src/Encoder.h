/*
 * Encoder.h
 *
 *
 *      Author: UG
 */

#ifndef SRC_ENCODER_H_
#define SRC_ENCODER_H_

#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define SPI_CMD_READ 0x4000
#define SPI_REG_AGC 0x3ffd
#define SPI_REG_MAG 0x3ffe
#define SPI_REG_DATA 0x3fff
#define SPI_REG_CLRERR 0x1

float QEnc_Update(int enc_channel,float Ts,float gear);
float getMechanicalAngle(int channel);
float AS5048A_GetAngle();
float spiReadData();
uint16_t AS5048A_ReadRaw(void);

#endif /* SRC_ENCODER_H_ */
