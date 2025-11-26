/*
 * ProcessCmd.c
 *
 *
 *      Author: UG
 */
#include "ProcessCmd.h"
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include "DC_Motor.h"
#include "BLDC.h"
#include "MotorIdent.h"
#include "StepMotor.h"

uint8_t cmd[64];
MCCommand RecvCMD;
uint8_t CommanStatus=0;
uint8_t activate_sys=0;


void Init_motor(){

	if(Check_Motor_Type_Channel(&RecvCMD)!=1) // check motor type, and determine active channel
		return; // open or unmatched motor

	switch(RecvCMD.MotType){
		case 0:    // DC Motor
			DC_Init(&RecvCMD);
		break;
		case 1:    // Step Motor
			StepperLUT_Init(&RecvCMD);
		break;
		case 2:    // BLDC motor
			BLDC_Init(&RecvCMD);
		break;

	}

}
void execute_command(){

	switch ( RecvCMD.Opcode) {
		case OP_SET_MOT_TYPE:   // 0: DC 1: Step 2: BLDC
			Init_motor();
			break;
		case OP_SET_RUN:
			activate_sys = 1;
			break;
		case OP_SET_STOP:
			activate_sys = 0;
			break;


	}

}

void ChechCMD(){
	uint8_t Checksum=0;
	int i=0;

	CommanStatus = NOEXEC_COMMAND;
	if(trcCDCReceive(cmd)==TRUE){

		if(cmd[0]==START_DELIM){
		   Checksum=0;
		   for(i=0; i<57;i++) Checksum+=cmd[i];
		   if(Checksum == cmd[57]){  //Check checksum
			   memcpy((uint8_t *)&RecvCMD,&cmd[1],sizeof(MCCommand)); // Load command
			   CommanStatus = EXEC_COMMAND;
		   }

		}
	}

	if(CommanStatus == EXEC_COMMAND)
		execute_command();

}


