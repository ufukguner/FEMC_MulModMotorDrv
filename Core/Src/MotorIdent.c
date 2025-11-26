/*
 * CombineDrv.c
 *
 *
 *      Author: UG
 */

#include "MotorIdent.h"

#include "math.h"
#include "Parameter.h"
#include "adc_dma.h"
#include "usbd_cdc_if.h"


#define SQRT3   1.73205080757f
#define TWO_PI  (2.0f * M_PI)

extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1, htim2, htim3, htim5, htim8, htim10;

ComDrv_Par_t M_par;

void TIM2_SetMode_Up(void)
{
    // Up counting
    TIM2->CR1 &= ~TIM_CR1_CMS;
}
void TIM2_SetMode_Center(void)
{
    // Center-aligned mode 1
    TIM2->CR1 &= ~TIM_CR1_CMS;
    TIM2->CR1 |=  (1 << TIM_CR1_CMS_Pos);
}
int Check_Motor_Type_Channel(MCCommand *RecvCMD)
{
    float Ir[4] = {0};
    float Ig[4][4] = {0};
    int Bm[4][4] = {0};          // Current response matrix , only diagonal element
    int rcnt = 0;          // her dürtüde tepki sayısı
    int i, j;
    float delta_e = 0.5;

    // Current limit
    float vref = 0.2; // Referance current
    uint32_t Ipk = (uint32_t)((vref / 3.3f) * 4095.0f);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Ipk);

    // Calculate relative threshold
    float Ipk_ref = vref*delta_e;

    // Prepare driver
    DRV8962_Sleep_OFF();
    DRV8962_Disable_ALL();
    DRV8962_Enable_ALL();
    StartDMAADC();
    HAL_Delay(50);  // Necessary for DMA filter
    TIM2_SetMode_Up(); // Timer must be counter up mode
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    htim2.Instance->ARR=3360;  // 25 kHz
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1;
    uint32_t zero   = 0;
    uint32_t dA     = (uint32_t)(period * 0.35f);  // calculate test duty

    // Check channels
    for (i = 0; i < 4; i++)
    {
        //set channels to zero
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1 , zero);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2 , zero);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3 , zero);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4 , zero);

        // excite one channel
        switch (i) {
			case 0:
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1 , dA);
				break;
			case 1:
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2 , dA);
				break;
			case 2:
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3 , dA);
				break;
			case 3:
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4 , dA);
				break;
			default:
				break;
		}

        for(int p=0;p<4;p++){
        	HAL_Delay(2); // wait for ADC_DMA
        	Read_ADC_FDMA(Ir);
        }

        for (j = 0; j < 4; j++) Ig[i][j] = Ir[j] ;
    }

    //Colse channel
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1 , zero);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2 , zero);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3 , zero);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4 , zero);

    // Disable driver
    DRV8962_Disable_ALL();

    // Calculate current response matrix
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
        	if (Ig[i][j] >= Ipk_ref) { Bm[i][j] = 1; rcnt++; }
            else Bm[i][j] = 0;
        }
    }

    // Check motor type
    switch (rcnt) {
		case 0:  // no connection
			M_par.Mot_Type = -1;

			break;
		case 1:  // problem
			M_par.Mot_Type = -1;

			break;
		case 4:   // Step Motor
			M_par.Mot_Type = 1;
			M_par.Active_Ch[0]=TIM_CHANNEL_1;
			M_par.Active_Ch[1]=TIM_CHANNEL_2;
			M_par.Active_Ch[2]=TIM_CHANNEL_3;
			M_par.Active_Ch[3]=TIM_CHANNEL_4;
			break;
		case 3:   // BLDC motor
			uint32_t rm[3];
			rm[0]=0; rm[1]=0; rm[2]=0; rm[3]=0;
			int p=0;
			M_par.Mot_Type = 2;
			for(i=0;i<4;i++){
				if(Bm[i][i]==1){
					rm[p]=i*4;
					p++;
				}

			}
			M_par.Active_Ch[0]=rm[0];
			M_par.Active_Ch[1]=rm[1];
			M_par.Active_Ch[2]=rm[2];
			break;
		case 2:   // DC motor, two DC motor should not be connected
			M_par.Mot_Type = 0;
		    rm[0]=0; rm[1]=0; rm[2]=0; rm[3]=0;
			p=0;
			for(i=0;i<4;i++){
				if(Bm[i][i]==1){
					rm[p]=i*4;
					p++;
				}

			}
			M_par.Active_Ch[0]=rm[0];
			M_par.Active_Ch[1]=rm[1];

			break;
		default:
			break;
	}

    // Compare motor type and user selection
    if(M_par.Mot_Type != RecvCMD->MotType)
    	return -1;

    TIM2_SetMode_Center();
    return 1;
}

