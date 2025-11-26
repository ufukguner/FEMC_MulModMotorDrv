/*
 * ProcessCmd.h
 *
 *
 *      Author: UG
 */

#ifndef PROCESSCMD_H_
#define PROCESSCMD_H_
#include "stm32f4xx_hal.h"
#define TRUE     1
#define FALSE    0

//packet
#define START_DELIM       30
#define EXEC_COMMAND      1
#define NOEXEC_COMMAND    0

typedef struct
{
  uint8_t Opcode;
  uint8_t MotType;
  uint8_t H_Bridge;
  uint8_t Encoder;
  float Vel_Set;
  float Pos_Set;
  float Kp;
  float Ki;
  float Kd;
  float Ipk;
  uint32_t freq;
  uint32_t S_Num;  //36 byte
  int Microstep;
  int Polepair;
  float Gain;
  float StepSpeed;
  float GearRatio;   //56 byte

} MCCommand;



typedef struct __attribute__((packed)) {
	uint8_t stx;
	float I1;
	float I2;
	float I3;
	float I4;
	float Vbus;
	float Vel_QENC;
	float Pos_MagENC;
	int status;
	int SaveStat;
	uint8_t etx;
} F_msg;

enum {
    OP_SET_MOT_TYPE    	 = 0x01,
	OP_SET_RUN       	 = 0x02,
	OP_SET_STOP          = 0x03,
};
void ChechCMD();
#endif /* PROCESSCMD_H_ */
