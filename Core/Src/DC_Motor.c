/*
 * DC_Motor.c
 *
 *
 *      Author: UG
 */

#include "DC_Motor.h"
#include "math.h"
#include "Parameter.h"
#include "MotorIdent.h"
#include "adc_dma.h"
#include "usbd_cdc_if.h"




extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1, htim2, htim3, htim5, htim8, htim10;
extern ComDrv_Par_t M_par;

DC_Par_t DC;
F_msg DC_Msg;



// AS5048A read raw +pi,-pi
static inline float AS5048A_GetAngle_DPM(void){
    uint16_t raw = AS5048A_ReadRaw() & 0x3FFF;
    float ang = (float)raw * (TWO_PI / 16384.0f);
    if (ang > M_PI)
    	ang -= TWO_PI;
    return ang;
}

/*
 *  Direction
 */

void DC_Direction_Dedect(void)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2)+1;


    // Direction detection
    float p_pos1 = AS5048A_GetAngle_DPM();

    float dA = 0.3* ((float)period);

    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[0],(uint32_t)dA);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[1],0);
    HAL_Delay(20);
    float p_pos2 = AS5048A_GetAngle_DPM();
    if(angle_diff(p_pos2,p_pos1) > 0)
     	 DC.enc_dir = 1;
    else
    	 DC.enc_dir = -1;

    // Set channels to zero
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[0],0);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[1],0);

}


void DC_Init(MCCommand *RecvCMD)
{
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

    float vref = RecvCMD->Ipk * 0.22f;
    uint32_t code = (uint32_t)((vref / 3.3f) * 4095.0f);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code);


    DRV8962_Sleep_OFF();
    DRV8962_Disable_ALL();
    DRV8962_Enable_Pin(M_par.Active_Ch[0]);
    DRV8962_Enable_Pin(M_par.Active_Ch[1]);
    TIM2_SetMode_Up();
    HAL_TIM_PWM_Start(&htim2, M_par.Active_Ch[0]);
    HAL_TIM_PWM_Start(&htim2, M_par.Active_Ch[1]);

    DC.encoder = RecvCMD->Encoder;
    if(DC.encoder == 0){
        HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(&htim1,0);
    }
    if(DC.encoder == 1){
        HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(&htim8,0);
    }

    DC.Enable = 1;
    DC.Con_frq = RecvCMD->freq;           // micro second
    DC.rate = (float)DC.Con_frq/1000000;  // second
    DC.Gear = RecvCMD->GearRatio;
    DC.Ref_Vel = RecvCMD->Vel_Set;
    DC.Ref_Pos = RecvCMD->Pos_Set;
    DC.Kp = RecvCMD->Kp;
    DC.Ki = RecvCMD->Ki;
    DC.Kd = RecvCMD->Kd;
    DC.Sample = RecvCMD->S_Num;
    //init variable
    DC.Sample_Cnt = 0;
    DC.Integral = 0;

    StartDMAADC();
    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_Base_Start(&htim10);
}


/* ---------------------- Ana Güncelleme Döngüsü ---------------------- */
void DC_Motor_Con()
{

	if(DC.Enable != 1)
		return;

    uint32_t con = __HAL_TIM_GET_COUNTER(&htim5);
    if(con >= DC.Con_frq){
    	__HAL_TIM_SET_COUNTER(&htim5,0);
    	float Ir[5];
    	Read_ADC_FDMA(&Ir[0]);
    	DC.Pos = AS5048A_GetAngle_DPM()*DC.enc_dir;

        // PID Controller
        float err = DC.Ref_Pos - DC.Pos;
        DC.Integral = DC.Integral+err*DC.rate;
        float Derivative = 	err-DC.PreErr;
        float u_pos = err*DC.Kp+DC.Integral*DC.Ki+Derivative*DC.Kd;
        u_pos = clampf(u_pos, -1.0f, 1.0f);

        float I_ref = u_pos * DC.Imax;

		float err_I = I_ref - Ir[0];

		DC.I_Integral += err_I * DC.rate;
		DC.I_Integral = clampf(DC.I_Integral, -5, 5);

		float u_I =
			err_I * DC.I_Kp +DC.I_Integral * DC.I_Ki;

		// PWM clamp
		u_I = clampf(u_I, -1.0f, 1.0f);


        //PWM ratio
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
        if (u_I >= 0){
            __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[0], (uint32_t)(u_I  * arr));
            __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[1], 0);
        } else {
            __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[0], 0);
            __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[1], (uint32_t)(-u_I  * arr));
        }
    }

    uint32_t msg_cnt = __HAL_TIM_GET_COUNTER(&htim10);
    if(msg_cnt >= MSG_RATE){
     	__HAL_TIM_SET_COUNTER(&htim10,0);
     	DC.Vel = QEnc_Update(DC.encoder,DC.rate,DC.Gear)*DC.enc_dir;
     	DC.Sample_Cnt++;   // Sample counter
     	DC_Msg.stx = STX;  //Preamp
     	DC_Msg.etx = ETX;  // end
     	DC_Msg.Vel_QENC = DC.Vel;  // measured velocity
     	DC_Msg.Pos_MagENC = DC.Pos; // position measurement
     	DC_Msg.status = DRV8962_Read_FAULT(); //Check driver faults
     	float Ir[5];
     	Read_ADC_FDMA(&Ir[0]);
    	DC_Msg.I1 = Ir[0];DC_Msg.I2 = Ir[1];DC_Msg.I3 = Ir[2];DC_Msg.I4 = Ir[3];DC_Msg.Vbus = Ir[4];
    	DC_Msg.SaveStat = 1;
    	// Disable controller for Sample complete or driver fault
       	if(DC.Sample_Cnt>=DC.Sample || DC_Msg.status == 1){
       		DC_Msg.SaveStat = 999;
       		DC.Enable = 0;
       		StopDMAADC();
       		DRV8962_Disable_ALL();
       		DRV8962_Sleep_ON();
       		SendUSBData((uint8_t *)&DC_Msg, sizeof(F_msg));
       	}
       	SendUSBData((uint8_t *)&DC_Msg, sizeof(F_msg));
    }



}


