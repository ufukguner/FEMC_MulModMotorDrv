################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BLDC.c \
../Core/Src/DC_Motor.c \
../Core/Src/Encoder.c \
../Core/Src/MotorIdent.c \
../Core/Src/Parameter.c \
../Core/Src/ProcessCmd.c \
../Core/Src/StepMotor.c \
../Core/Src/adc_dma.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/BLDC.o \
./Core/Src/DC_Motor.o \
./Core/Src/Encoder.o \
./Core/Src/MotorIdent.o \
./Core/Src/Parameter.o \
./Core/Src/ProcessCmd.o \
./Core/Src/StepMotor.o \
./Core/Src/adc_dma.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/BLDC.d \
./Core/Src/DC_Motor.d \
./Core/Src/Encoder.d \
./Core/Src/MotorIdent.d \
./Core/Src/Parameter.d \
./Core/Src/ProcessCmd.d \
./Core/Src/StepMotor.d \
./Core/Src/adc_dma.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BLDC.cyclo ./Core/Src/BLDC.d ./Core/Src/BLDC.o ./Core/Src/BLDC.su ./Core/Src/DC_Motor.cyclo ./Core/Src/DC_Motor.d ./Core/Src/DC_Motor.o ./Core/Src/DC_Motor.su ./Core/Src/Encoder.cyclo ./Core/Src/Encoder.d ./Core/Src/Encoder.o ./Core/Src/Encoder.su ./Core/Src/MotorIdent.cyclo ./Core/Src/MotorIdent.d ./Core/Src/MotorIdent.o ./Core/Src/MotorIdent.su ./Core/Src/Parameter.cyclo ./Core/Src/Parameter.d ./Core/Src/Parameter.o ./Core/Src/Parameter.su ./Core/Src/ProcessCmd.cyclo ./Core/Src/ProcessCmd.d ./Core/Src/ProcessCmd.o ./Core/Src/ProcessCmd.su ./Core/Src/StepMotor.cyclo ./Core/Src/StepMotor.d ./Core/Src/StepMotor.o ./Core/Src/StepMotor.su ./Core/Src/adc_dma.cyclo ./Core/Src/adc_dma.d ./Core/Src/adc_dma.o ./Core/Src/adc_dma.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

