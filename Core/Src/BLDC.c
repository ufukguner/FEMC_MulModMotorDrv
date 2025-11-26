/*
 * BLDC.h
 *
 *
 *      Author: UG
 */

#include "BLDC.h"
#include "math.h"
#include "Parameter.h"
#include "adc_dma.h"
#include "usbd_cdc_if.h"
#include "MotorIdent.h"

#define SQRT3   1.73205080757f


extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1, htim2, htim3, htim5, htim8, htim10;

extern ComDrv_Par_t M_par;

float Kp_id = 0.5f, Ki_id = 0.01f;
float Kp_iq = 5.0f, Ki_iq = 0.05f;
float Kp_pos = 6.0f, Ki_pos = 0.01f;
float Ipos_MAX = 0.3f;

// Internal control params
static float id_int=0, iq_int=0, pos_int=0;
float err_pos=0, control_u=0, pos_derv=0;


BLDC_Par_t BLDC; // Motor structure
F_msg BLDC_Msg;  // Message structure



// Wrap check
static inline float wrap_pm_pi(float x){
    x = fmodf(x, TWO_PI);
    if (x >  M_PI) x -= TWO_PI;
    if (x < -M_PI) x += TWO_PI;
    return x;
}


// AS5048A raw read (+pi,-pi)
static inline float AS5048A_GetAngle_PM(void){
    uint16_t raw = AS5048A_ReadRaw() & 0x3FFF;
    float ang = (float)raw * (TWO_PI / 16384.0f);
    if (ang > M_PI) ang -= TWO_PI;
    return ang;
}

// Electrical angle
static inline float getElectricalAngle(void){
    float mech = AS5048A_GetAngle_PM();
    float elec = (mech - BLDC.enc_offset) * BLDC.PolePair * BLDC.enc_dir;
    return wrap_pm_pi(elec);
}

// Aligned mechanical angle
static inline float mech_aligned_rad(void){
    float mech = AS5048A_GetAngle_PM();
    return wrap_pm_pi((mech - BLDC.enc_offset) * BLDC.enc_dir);
}

/*
 *  Initialize BLDC
 *
 * */

void BLDC_Init(MCCommand *RecvCMD)
{


    // Set current limit
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    float vref = RecvCMD->Ipk;
    uint32_t Ipk = (uint32_t)((vref / 3.3f) * 4095.0f);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Ipk);

    // Initilaze  DRV8962
    DRV8962_Sleep_OFF();
    DRV8962_Disable_ALL();
    DRV8962_Enable_Pin(M_par.Active_Ch[0]);
    DRV8962_Enable_Pin(M_par.Active_Ch[1]);
    DRV8962_Enable_Pin(M_par.Active_Ch[2]);

    htim2.Instance->ARR = 3360;  //Set PWM frequency to 25 Khz

    HAL_TIM_PWM_Start(&htim2, M_par.Active_Ch[0]);
    HAL_TIM_PWM_Start(&htim2, M_par.Active_Ch[1]);
    HAL_TIM_PWM_Start(&htim2, M_par.Active_Ch[2]);

    BLDC.encoder = RecvCMD->Encoder;
    if (BLDC.encoder==0) {
    	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    	__HAL_TIM_SET_COUNTER(&htim1,0);
    }
    if (BLDC.encoder==1){
    	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    	__HAL_TIM_SET_COUNTER(&htim8,0);
    }

    BLDC_InitPosition();   // Direction and magnetic zero detection

    BLDC.Enable = 1;
    BLDC.Con_frq = RecvCMD->freq;
    BLDC.rate = (float)BLDC.Con_frq/1000000;
    BLDC.Gear = RecvCMD->GearRatio;
    BLDC.Ref_Pos = wrap_pm_pi(RecvCMD->Pos_Set);
    BLDC.PolePair = RecvCMD->Polepair;
    BLDC.Sample_Cnt = 0;
    BLDC.Sample = RecvCMD->S_Num;
    BLDC.Integral = 0;
    BLDC.Kp =RecvCMD->Kp;
    BLDC.Ki =RecvCMD->Ki;
    BLDC.Kd =RecvCMD->Kd;
    //init variable
    id_int=0;
    iq_int=0;
    pos_int=0;
    StartDMAADC();
    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_Base_Start(&htim10);

}

/*
 *  Direction and magnetic zero
 */

void BLDC_InitPosition(void)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2)+1;
    float half = 0.5f * period;
    float vd=0.35f, vq=0.0f;
    float c=1.0f, s=0.0f;

    float valpha = vd*c - vq*s;
    float vbeta  = vd*s + vq*c;
    float dA = half +  valpha * half;
    float dB = half + ((-0.5f*valpha)+(SQRT3*0.5f*vbeta))*half;
    float dC = half + ((-0.5f*valpha)-(SQRT3*0.5f*vbeta))*half;

    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[0],(uint32_t)dA);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[1],(uint32_t)dB);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[2],(uint32_t)dC);

    HAL_Delay(800); // Wait for rotor aligned to d axis

    // Average for magnetic zero
    float sum=0;
    for(int i=0;i<10;i++){ sum += AS5048A_GetAngle_PM(); HAL_Delay(2); }
    BLDC.enc_offset = sum/10.0f;

    // Direction detection
    float p_pos1 = AS5048A_GetAngle_PM();
    vq = +0.05f;
    valpha = -vq*s;
    vbeta = +vq*c;
    dA = half + valpha*half;
    dB = half + ((-0.5f*valpha)+(SQRT3*0.5f*vbeta))*half;
    dC = half + ((-0.5f*valpha)-(SQRT3*0.5f*vbeta))*half;
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[0],(uint32_t)dA);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[1],(uint32_t)dB);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[2],(uint32_t)dC);
    HAL_Delay(20);
    float p_pos2 = AS5048A_GetAngle_PM();
    if(angle_diff(p_pos2,p_pos1) > 0)
     	 BLDC.enc_dir = 1;
    else
    	 BLDC.enc_dir = -1;

    // Set channels to center
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[0],(uint32_t)half);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[1],(uint32_t)half);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[2],(uint32_t)half);

}

/*
 * Current control
 *
 *
 *  */
void FOC_Update(float id_ref, float iq_ref)
{
    float Ir[5]; Read_ADC_FDMA(Ir);
    float ia=Ir[0], ib=Ir[1], vbus=Ir[4];


    // Clarke
    float alpha = ia;
    float beta = (ia+2.0f*ib)/SQRT3;

    // Park
    float theta = getElectricalAngle();
    float c=cosf(theta), s=sinf(theta);
    float id= alpha*c + beta*s;
    float iq=-alpha*s + beta*c;

    // PI controller
    float err_id=id_ref-id;
    float err_iq=iq_ref-iq;
    id_int += Ki_id*err_id;
    iq_int += Ki_iq*err_iq;

    // anti-windup
    id_int = fminf(fmaxf(id_int,-3.0f),3.0f);
    iq_int = fminf(fmaxf(iq_int,-3.0f),3.0f);
    float vd=Kp_id*err_id+id_int;
    float vq=Kp_iq*err_iq+iq_int;

    // Limit
    float vlim=0.577f*vbus;
    float vmag=sqrtf(vd*vd+vq*vq);
    if(vmag>vlim){ float s=vlim/vmag; vd*=s; vq*=s; }

    // Inverse Park
    float valpha=vd*c - vq*s;
    float vbeta =vd*s + vq*c;

    uint32_t period=__HAL_TIM_GET_AUTORELOAD(&htim2)+1;
    float half=0.5f*period;
    float dA=half+(valpha/vbus)*half;
    float dB=half+(((-0.5f*valpha)+(SQRT3*0.5f*vbeta))/vbus)*half;
    float dC=half+(((-0.5f*valpha)-(SQRT3*0.5f*vbeta))/vbus)*half;
    dA=fminf(fmaxf(dA,0),period);
    dB=fminf(fmaxf(dB,0),period);
    dC=fminf(fmaxf(dC,0),period);

    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[0],(uint32_t)dA);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[1],(uint32_t)dB);
    __HAL_TIM_SET_COMPARE(&htim2,M_par.Active_Ch[2],(uint32_t)dC);
}

/*
 * FOC Position Control Loop
 *
 *
 *  */
void BLDC_Motor_Con(void)
{
    if(BLDC.Enable!=1) return;

    uint32_t con=__HAL_TIM_GET_COUNTER(&htim5);
    if(con>=BLDC.Con_frq){
        __HAL_TIM_SET_COUNTER(&htim5,0);

        float theta_mech=mech_aligned_rad();
        BLDC.Pos=theta_mech;

        // Integral
        err_pos=angle_diff(BLDC.Ref_Pos,theta_mech);
        pos_int += err_pos;
        if(pos_int>Ipos_MAX) pos_int=Ipos_MAX;
        if(pos_int<-Ipos_MAX) pos_int=-Ipos_MAX;

        // Derivative
        pos_derv = err_pos -BLDC.PreErr;
        BLDC.PreErr = err_pos;

        // PID controller
        float iq_ref=  BLDC.Kp*err_pos +  BLDC.Ki*pos_int+BLDC.Kd*pos_derv;
        iq_ref=fminf(fmaxf(iq_ref,-0.8f),0.8f);
        control_u=iq_ref;

        // Current regulation of FOC
        FOC_Update(0.0f,iq_ref);
    }

    uint32_t msg_cnt=__HAL_TIM_GET_COUNTER(&htim10);
    if(msg_cnt>=MSG_RATE){
        __HAL_TIM_SET_COUNTER(&htim10,0);
        BLDC.Vel=QEnc_Update(BLDC.encoder,BLDC.rate,BLDC.Gear)*BLDC.enc_dir;
        BLDC.Sample_Cnt++;
        BLDC_Msg.stx=STX;
        BLDC_Msg.etx=ETX;
        BLDC_Msg.Pos_MagENC=BLDC.Pos;
        BLDC_Msg.Vel_QENC=BLDC.Vel;
        BLDC_Msg.status = DRV8962_Read_FAULT(); // check driver fault
        float Ir[5]; Read_ADC_FDMA(Ir);
        BLDC_Msg.I1=Ir[0]; BLDC_Msg.I2=Ir[1];
        BLDC_Msg.I3=Ir[2]; BLDC_Msg.I4=Ir[3];
        BLDC_Msg.Vbus=Ir[4];
        BLDC_Msg.SaveStat=1;
        // Disable controller for Sample complete or driver fault
    	if(BLDC.Sample_Cnt>=BLDC.Sample || BLDC_Msg.status == 1){
			BLDC_Msg.SaveStat = 999;
			BLDC.Enable = 0;
			StopDMAADC();
			DRV8962_Disable_ALL();
			DRV8962_Sleep_ON();
			SendUSBData((uint8_t *)&BLDC_Msg, sizeof(F_msg));
		}

    	SendUSBData((uint8_t*)&BLDC_Msg,sizeof(F_msg));
    }
}
