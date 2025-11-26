/*
 * Encoder.h
 *
 *
 *      Author: UG
 */

#include "Encoder.h"
#include <math.h>

#define ENC_LINES        4000U  //Dual interrupt one reolution encoder output

#define SQRT3            1.73205080757f
#define TWO_PI           (2.0f * M_PI)

extern SPI_HandleTypeDef hspi2;

#define ENC_ALPHA  0.1f

typedef struct {
    int32_t last_cnt;
    float speed_rps;   // rad/s
    float last_speed;
} QEnc_t;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

uint16_t magnitude;
static uint16_t last_cnt = 0;
static int32_t  pos_accum = 0;
float thetaf=0;
QEnc_t qenc[2];   // optic encoder struct

float QEnc_Update(int enc_channel, float Ts, float gear)
{
    TIM_HandleTypeDef *htim = (enc_channel == 0) ? &htim1 : &htim8;
    QEnc_t *qe = &qenc[enc_channel];

    // current value
    int32_t now = (int32_t)__HAL_TIM_GET_COUNTER(htim);

    // Difference
    int32_t diff = now - qe->last_cnt;
    qe->last_cnt = now;

    // Overflow check
    if (diff >  32768) diff -= 65536;
    if (diff < -32768) diff += 65536;

    // Calculate rps
    float rev_per_s = (float)diff / (ENC_LINES * Ts * gear);

    // rad/s
    qe->speed_rps = rev_per_s * 2.0f * (float)M_PI;
    qe->speed_rps = ENC_ALPHA * qe->speed_rps + (1.0f - ENC_ALPHA) * qe->last_speed;
    qe->last_speed = qe->speed_rps;

    return qe->speed_rps;
}


float getMechanicalAngle(int channel) {
	uint16_t cnt =0;
	if(channel == 0)
		cnt = __HAL_TIM_GET_COUNTER(&htim1);
	else
		cnt = __HAL_TIM_GET_COUNTER(&htim8);

    int16_t diff = (int16_t)(cnt - last_cnt);
    pos_accum += diff;
    last_cnt = cnt;
    float mech_revs = (float)pos_accum / (float)ENC_LINES;
    float theta = mech_revs * TWO_PI;
    thetaf = theta;
    theta = fmodf(theta, TWO_PI);
    if (theta < 0) theta += TWO_PI;
    return theta;
}


static uint8_t spiCalcEvenParity(uint16_t value) {
	  uint8_t cnt = 0;
	  uint8_t i;
	  for (i = 0; i < 16; i++)
	  {
	  if (value & 0x1)
	  {
	  cnt++;
	  }
	  value >>= 1;
	  }
	  return cnt & 0x1;
 }

uint16_t spiTransfer(uint8_t  *dat , int len){
    uint8_t mn[2];
    uint16_t res=0;
    uint8_t ht[2];
    ht[0]=dat[1];
    ht[1]=dat[0];
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2,ht,(uint8_t*)&mn[0],2,0xfff);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	res =(mn[0]<<8)|(mn[1]);
	return res;
}
float AS5048A_GetAngle()
{
  uint16_t dat; // 16-bit data buffer for SPI communication
  uint16_t  agcreg, val,magreg;

 // Send READ MAG command.
 dat = SPI_CMD_READ | SPI_REG_MAG;
 dat |= spiCalcEvenParity(dat) << 15;
 agcreg=spiTransfer((uint8_t*)&dat, 2);


 // Send READ ANGLE command.
 dat = 0xffff;
 dat |= spiCalcEvenParity(dat) << 15;
 magreg= spiTransfer((uint8_t*)&dat, 2);

 // Send NOP command.
 dat = 0x0000; // NOP command.
 val = spiTransfer((uint8_t*)&dat, 2);

 if ((val & 0x4000))  // if there is a error
  {

	  dat = SPI_CMD_READ | SPI_REG_CLRERR;
	  dat |= spiCalcEvenParity(dat)<<15;
	  spiTransfer((uint8_t*)&dat, 2);
	  dat = 0x0000; // NOP command.
	  val = spiTransfer((uint8_t*)&dat, 2);
  }


magnitude = magreg & (16384 - 31 - 1);
return (val & 0x3FFF) * (TWO_PI / 16384.0f) ; //radian

}
uint16_t AS5048A_ReadRaw(void)
{
    uint16_t dat;
    uint16_t val, magreg, agcreg;

    // Read sequence  AS5048A
    dat = SPI_CMD_READ | SPI_REG_MAG;
    dat |= spiCalcEvenParity(dat) << 15;
    agcreg = spiTransfer((uint8_t*)&dat, 2);

    dat = 0xFFFF;
    dat |= spiCalcEvenParity(dat) << 15;
    magreg = spiTransfer((uint8_t*)&dat, 2);

    dat = 0x0000;  // NOP
    val = spiTransfer((uint8_t*)&dat, 2);

    // Error check
    if (val & 0x4000)
    {
        dat = SPI_CMD_READ | SPI_REG_CLRERR;
        dat |= spiCalcEvenParity(dat) << 15;
        spiTransfer((uint8_t*)&dat, 2);
        dat = 0x0000;
        val = spiTransfer((uint8_t*)&dat, 2);
    }

    uint16_t angle_raw = val & 0x3FFF;   // 0..16383
    return angle_raw;
}


