/*
 * adc_dma.h
 *
 *
 *      Author: UG
 */

#ifndef SRC_ADC_DMA_H_
#define SRC_ADC_DMA_H_

#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>


void Read_ADC_FDMA(float *HB_Cur);
float ReadBusVolatge();
void StartDMAADC();
void StopDMAADC();
#endif /* SRC_ADC_DMA_H_ */
