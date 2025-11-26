/*
 * StepMotor.c
 *
 *
 *      Author: UG
 */


#include "StepMotor.h"
#include "Parameter.h"
#include "ProcessCmd.h"
#include "adc_dma.h"
#include "Encoder.h"
#include "MotorIdent.h"
#include "usbd_cdc_if.h"


extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1, htim2, htim3, htim5, htim8, htim10;

extern ComDrv_Par_t M_par;

#define MAX_MICROSTEPS 128
float lut_sin[MAX_MICROSTEPS];
float lut_cos[MAX_MICROSTEPS];

float step_angle;       // bir mikrostepin radyan karşılığı
float pos_enc;          // gerçek açı [rad]
float angle_cmd;        // hedef açı [rad]

uint32_t last_tick;
uint8_t enabled;

Step_Par_t Step;
F_msg Step_Msg;



static uint16_t N;

// Create Look Up Table
float GenerateLUT(int microsteps)
{
    N = microsteps;
    float dphi = HALF_PI / (float)N;   // çeyrek çözünürlük
    for (uint16_t i=0; i<N; i++) {
        lut_sin[i] = sinf(i * dphi);   // 0..π/2 → hep +
    }

    return 0;
}

// AS5048A read raw +pi,-pi
static inline float AS5048A_GetAngle_SPM(void){
    uint16_t raw = AS5048A_ReadRaw() & 0x3FFF;
    float ang = (float)raw * (TWO_PI / 16384.0f);
    if (ang > M_PI)
    	ang -= TWO_PI;
    return ang;
}

/*
 *  Rotation detection
 */
void Step_Direction_Dedect(void)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2)+1;


    // Direction detection
    float p_pos1 = AS5048A_GetAngle_SPM();

    float dA = 0.3* ((float)period);
    float dB = (1-0.3)* ((float)period);

    __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[0], dA);
    __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[1], 0);
    __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[2], dB);
 	__HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[3], 0);

    HAL_Delay(20);
    float p_pos2 = AS5048A_GetAngle_SPM();
    if(angle_diff(p_pos2,p_pos1) > 0)
    	Step.enc_dir = 1;
    else
    	Step.enc_dir = -1;

    // Set channels to zero
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[0],0);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[1],0);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[2],0);
 	__HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[3],0);

}

// Initialize Step motor
void StepperLUT_Init(MCCommand *RecvCMD)
{

    angle_cmd = 0.0f;


    Step.encoder = RecvCMD->Encoder;
	if(Step.encoder == 0){
		 HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	}
	if(Step.encoder == 1){
		 HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	}

    DRV8962_Sleep_OFF();
    DRV8962_Disable_ALL();

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    float vref = RecvCMD->Ipk ;
    uint32_t code = (uint32_t)((vref / 3.3f) * 4095.0f);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code);

    DRV8962_Enable_Pin(M_par.Active_Ch[0]);
    DRV8962_Enable_Pin(M_par.Active_Ch[1]);
    DRV8962_Enable_Pin(M_par.Active_Ch[2]);
    DRV8962_Enable_Pin(M_par.Active_Ch[3]);

    HAL_TIM_Base_Start(&htim2);
    htim2.Instance->ARR = 2799;   // 30Khz PWM

    HAL_TIM_PWM_Start(&htim2, M_par.Active_Ch[0]);
    HAL_TIM_PWM_Start(&htim2, M_par.Active_Ch[1]);
    HAL_TIM_PWM_Start(&htim2, M_par.Active_Ch[2]);
    HAL_TIM_PWM_Start(&htim2, M_par.Active_Ch[3]);

    Step_Direction_Dedect();// detect direction

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2)+1;
    float half = 0.5f * period;

    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[0],(uint32_t)half);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[1],(uint32_t)half);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[2],(uint32_t)half);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[3],(uint32_t)half);

    Step.Enable = 1;
    Step.Con_frq = RecvCMD->freq;
    Step.rate = (float)Step.Con_frq/1000000;
    Step.Gear = RecvCMD->GearRatio;
    Step.Ref_Vel = RecvCMD->Vel_Set;
    Step.Ref_Pos = RecvCMD->Pos_Set;
    Step.Kp = RecvCMD->Kp;
    Step.Ki = RecvCMD->Ki;
    Step.Kd = RecvCMD->Kd;
    Step.Sample = RecvCMD->S_Num;
    Step.Sample_Cnt = 0;
    Step.Gain = RecvCMD->Gain;
    Step.Microstep = RecvCMD->Microstep;
    Step.Step_Speed = RecvCMD->StepSpeed;
    //init variable
    Step.PreErr=0;
    Step.Integral=0;

    //Generate Look Up Table
    GenerateLUT(Step.Microstep);
    StartDMAADC();

    // start control loop  and message loop timer
    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_Base_Start(&htim10);
}


void Stepper_Con() {

	if(Step.Enable != 1)
		return;

	float Ir[5];
    uint32_t con = __HAL_TIM_GET_COUNTER(&htim5);
    if(con >= Step.Con_frq){
		__HAL_TIM_SET_COUNTER(&htim5,0);

	    Step.Pos = AS5048A_GetAngle_SPM();

        float error =  Step.Ref_Pos + Step.Pos;
        float deriv = (error-Step.PreErr);
        Step.Integral +=error*Step.rate;

        // anti-windup
        if (Step.Integral > 200.0f) Step.Integral = 200.0f;
        if (Step.Integral < -200.0f) Step.Integral = -200.0f;

        // Calculate referans velocity
        float w_ref = Step.Kp * error +Step.Ki * Step.Integral + Step.Kd * deriv;
        // Saturation
        if (w_ref > 200.0f)  w_ref = 200.0f;    // rad/s
        if (w_ref < -200.0f) w_ref = -200.0f;

        Step.PreErr = error;

        // Calculate for angle command
	    angle_cmd += w_ref  * Step.rate * TWO_PI;
	    if (angle_cmd >= 2.0f*M_PI) angle_cmd -= 2.0f*M_PI;
	    if (angle_cmd < 0.0f)       angle_cmd += 2.0f*M_PI;

	    // Calculate for pi/2
	    float theta = angle_cmd;
	    uint8_t q = (uint8_t)(theta / HALF_PI);
	    float   phi = theta - (float)q * HALF_PI;

	    // Calculate index for LUT
	    float    dphi = HALF_PI / (float)Step.Microstep;
	    uint16_t idx  = (uint16_t)(phi / dphi + 0.5f);
	    if (idx >= Step.Microstep) idx = Step.Microstep - 1;

	    // Calculate sin and cos value
	    float sQ = lut_sin[idx];
	    float cQ = lut_sin[(Step.Microstep-1) - idx];

	    // Arrange for angle change
	    float sinA, cosA;
	    switch (q) {
	      case 0: sinA= sQ;  cosA= cQ;  break;
	      case 1: sinA= cQ;  cosA=-sQ;  break;
	      case 2: sinA=-sQ;  cosA=-cQ;  break;
	      default:sinA=-cQ;  cosA= sQ;  break;
	    }

	    // Calculate duty
	    uint32_t arr   = __HAL_TIM_GET_AUTORELOAD(&STEP_PWM_TIMER);
	    uint32_t dutyA = (uint32_t)(fabsf(sinA) * Step.Gain * arr);
	    uint32_t dutyB = (uint32_t)(fabsf(cosA) * Step.Gain* arr);


	    // Apply PWM to the channels
	    if (sinA >= 0.0f) {
	        __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[0], dutyA);
	        __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[1], 0);
	    } else {
	        __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[0], 0);
	        __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[1], dutyA);
	    }

	    if (cosA >= 0.0f) {
	        __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[2], dutyB);
	        __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[3], 0);
	    } else {
	        __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[2], 0);
	        __HAL_TIM_SET_COMPARE(&htim2, M_par.Active_Ch[3], dutyB);
	    }
    }

    uint32_t msg_cnt = __HAL_TIM_GET_COUNTER(&htim10);
	if(msg_cnt >= MSG_RATE){
		__HAL_TIM_SET_COUNTER(&htim10,0);
		Step.Vel = QEnc_Update(Step.encoder,0.0025,Step.Gear);
		Step.Sample_Cnt++;
		Step_Msg.stx = STX;
		Step_Msg.etx = ETX;
		Step_Msg.Vel_QENC = Step.Vel;
		Step_Msg.Pos_MagENC = Step.Pos;
		Step_Msg.status = DRV8962_Read_FAULT(); // check driver fault
		Read_ADC_FDMA(&Ir[0]);
		Step_Msg.I1 = Ir[0];Step_Msg.I2 = Ir[1];Step_Msg.I3 = Ir[2];Step_Msg.I4 = Ir[3];Step_Msg.Vbus = Ir[4];
		Step_Msg.SaveStat = 1;
		// Disable controller for Sample complete or driver fault
		if(Step.Sample_Cnt>=Step.Sample || Step_Msg.status == 1){
			Step_Msg.SaveStat = 999;
			Step.Enable = 0;
			StopDMAADC();
			DRV8962_Disable_ALL();
			DRV8962_Sleep_ON();
			SendUSBData((uint8_t *)&Step_Msg, sizeof(F_msg));
		}
		SendUSBData((uint8_t *)&Step_Msg, sizeof(F_msg));
	}

}



