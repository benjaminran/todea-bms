################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/BMS1.c \
../src/LCD.c \
../src/Timer.c \
../src/main.c \
../src/stm320518_eval_i2c_ee.c \
../src/stm32f0xx_exti.c \
../src/stm32f0xx_i2c.c \
../src/stm32f0xx_it.c \
../src/stm32f0xx_misc.c \
../src/stm32f0xx_pwr.c \
../src/stm32f0xx_rtc.c \
../src/stm32f0xx_syscfg.c \
../src/stm32f0xx_tim.c \
../src/stm32f0xx_usart.c \
../src/tsl.c \
../src/tsl_acq.c \
../src/tsl_acq_stm32f0xx.c \
../src/tsl_dxs.c \
../src/tsl_ecs.c \
../src/tsl_filter.c \
../src/tsl_globals.c \
../src/tsl_linrot.c \
../src/tsl_object.c \
../src/tsl_time.c \
../src/tsl_time_stm32f0xx.c \
../src/tsl_touchkey.c \
../src/tsl_user.c 

OBJS += \
./src/BMS1.o \
./src/LCD.o \
./src/Timer.o \
./src/main.o \
./src/stm320518_eval_i2c_ee.o \
./src/stm32f0xx_exti.o \
./src/stm32f0xx_i2c.o \
./src/stm32f0xx_it.o \
./src/stm32f0xx_misc.o \
./src/stm32f0xx_pwr.o \
./src/stm32f0xx_rtc.o \
./src/stm32f0xx_syscfg.o \
./src/stm32f0xx_tim.o \
./src/stm32f0xx_usart.o \
./src/tsl.o \
./src/tsl_acq.o \
./src/tsl_acq_stm32f0xx.o \
./src/tsl_dxs.o \
./src/tsl_ecs.o \
./src/tsl_filter.o \
./src/tsl_globals.o \
./src/tsl_linrot.o \
./src/tsl_object.o \
./src/tsl_time.o \
./src/tsl_time_stm32f0xx.o \
./src/tsl_touchkey.o \
./src/tsl_user.o 

C_DEPS += \
./src/BMS1.d \
./src/LCD.d \
./src/Timer.d \
./src/main.d \
./src/stm320518_eval_i2c_ee.d \
./src/stm32f0xx_exti.d \
./src/stm32f0xx_i2c.d \
./src/stm32f0xx_it.d \
./src/stm32f0xx_misc.d \
./src/stm32f0xx_pwr.d \
./src/stm32f0xx_rtc.d \
./src/stm32f0xx_syscfg.d \
./src/stm32f0xx_tim.d \
./src/stm32f0xx_usart.d \
./src/tsl.d \
./src/tsl_acq.d \
./src/tsl_acq_stm32f0xx.d \
./src/tsl_dxs.d \
./src/tsl_ecs.d \
./src/tsl_filter.d \
./src/tsl_globals.d \
./src/tsl_linrot.d \
./src/tsl_object.d \
./src/tsl_time.d \
./src/tsl_time_stm32f0xx.d \
./src/tsl_touchkey.d \
./src/tsl_user.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wextra  -g -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F072 -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f0-stdperiph" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


