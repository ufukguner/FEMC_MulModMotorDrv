# FEMC_MulModMotorDrv
## Design and PCD
QuadDrv405_Schematic_PCB.pdf

## MATLAB Scripts

### `MotTest.m`
Sends motor control commands and parameters to the STM32 over USB CDC.

### `readSerialDataBytMot.m`
Serial callback function in MATLAB that receives, parses, and processes incoming data from the MCU.

---

## STM32 Firmware (MulMotDrv)

**Project:** `MulMotDrv`  
**Development Environment:** STM32CubeIDE **v1.16.1**

### Source Files (`Core/Src`)

MulMotDrv/Core/Src/


|-- adc_dma.c Current measurement (ADC + DMA)

|-- BLDC.c BLDC motor control algorithms (FOC)

|-- DC_Motor.c DC motor control procedures (PID+PI)

|-- StepMotor.c Step motor control procedures (LUT+PID)

|-- MotorIdent.c Motor type identification, active channels

|-- ProcessCmd.c USB CDC command parsing and handling

|-- Parameter.c DRV8962 driver functions 
