/*
 * adc_dma.c
 *
 *
 *      Author: UG
 */
#include "adc_dma.h"


#define VREF_VOLTAGE     3.3f
#define ADC_RESOLUTION   4095.0f
#define DIVIDER_RATIO    (105.6f/5.0f)
#define VBUS_MAX         12.0f

float   SHUNT_GAIN =       0.46f;  // Calculated for 3.3k
#define ADC2CUR_SCALE    (VREF_VOLTAGE / (ADC_RESOLUTION * SHUNT_GAIN))

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;  //ADC DMA trigger timer

static inline float adc_to_current(uint16_t c) {
    return c * ADC2CUR_SCALE;
}
#define NUM_CHANNELS     5 // ADC channel num
#define SAMPLES_PER_CH   10 // Filter sample
#define TOTAL_SAMPLES    (NUM_CHANNELS * SAMPLES_PER_CH)
#define MA_WINDOW_SIZE   10  // Moving window

// DMA buffer
uint16_t adc_dma_buffer[NUM_CHANNELS * MA_WINDOW_SIZE];

// Moving window buffer
uint16_t ma_buffer[NUM_CHANNELS][MA_WINDOW_SIZE];

// Average
uint32_t ma_sum[NUM_CHANNELS] = {0};

//Index
uint16_t ma_index = 0;

// Filtered measurement
volatile uint16_t ma_filtered[NUM_CHANNELS] = {0};

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        // Each DMA Complete comes with a new group MA_WINDOW_SIZE
        for(uint16_t i=0; i<MA_WINDOW_SIZE; i++)
        {
            for(uint8_t ch=0; ch<NUM_CHANNELS; ch++)
            {
                uint16_t sample = adc_dma_buffer[i*NUM_CHANNELS + ch];

                // Subtract the old value from the total
                ma_sum[ch] -= ma_buffer[ch][ma_index];

                // Add new value
                ma_sum[ch] += sample;

                // Save to Buffer
                ma_buffer[ch][ma_index] = sample;

                // Current average
                ma_filtered[ch] = (uint16_t)(ma_sum[ch]/MA_WINDOW_SIZE);
            }

            // Moving window advance
            ma_index++;
            if(ma_index >= MA_WINDOW_SIZE)
                ma_index = 0;
        }
    }
}
void Read_ADC_FDMA(float *HB_Cur){
	HB_Cur[0]= ADC2CUR_SCALE*ma_filtered[0];
	HB_Cur[1]= ADC2CUR_SCALE*ma_filtered[1];
	HB_Cur[2]= ADC2CUR_SCALE*ma_filtered[2];
	HB_Cur[3]= ADC2CUR_SCALE*ma_filtered[3];
	HB_Cur[4]=((float)ma_filtered[4] * VREF_VOLTAGE/ADC_RESOLUTION)*DIVIDER_RATIO;
}

float ReadBusVolatge(){
return ((float)ma_filtered[4] * VREF_VOLTAGE/ADC_RESOLUTION)*DIVIDER_RATIO;
}


void StartDMAADC(){
	 HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, NUM_CHANNELS * MA_WINDOW_SIZE);
     HAL_TIM_Base_Start_IT(&htim3);
}

void StopDMAADC(){
	 HAL_ADC_Stop_DMA(&hadc1);
     HAL_TIM_Base_Stop_IT(&htim3);
}
