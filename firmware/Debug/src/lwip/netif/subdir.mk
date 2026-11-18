################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/lwip/netif/bridgeif.c \
../src/lwip/netif/bridgeif_fdb.c \
../src/lwip/netif/ethernet.c \
../src/lwip/netif/lowpan6.c \
../src/lwip/netif/lowpan6_ble.c \
../src/lwip/netif/lowpan6_common.c \
../src/lwip/netif/slipif.c \
../src/lwip/netif/zepif.c 

OBJS += \
./src/lwip/netif/bridgeif.o \
./src/lwip/netif/bridgeif_fdb.o \
./src/lwip/netif/ethernet.o \
./src/lwip/netif/lowpan6.o \
./src/lwip/netif/lowpan6_ble.o \
./src/lwip/netif/lowpan6_common.o \
./src/lwip/netif/slipif.o \
./src/lwip/netif/zepif.o 

C_DEPS += \
./src/lwip/netif/bridgeif.d \
./src/lwip/netif/bridgeif_fdb.d \
./src/lwip/netif/ethernet.d \
./src/lwip/netif/lowpan6.d \
./src/lwip/netif/lowpan6_ble.d \
./src/lwip/netif/lowpan6_common.d \
./src/lwip/netif/slipif.d \
./src/lwip/netif/zepif.d 


# Each subdirectory must supply rules for building sources it contributes
src/lwip/netif/%.o: ../src/lwip/netif/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Andes C Compiler'
	$(CROSS_COMPILE)gcc -DportasmHANDLE_INTERRUPT=mext_interrupt -DREF_TEST_CLK -I"../src/bsp/ae350" -I"../src/bsp/config" -I"../src/bsp/driver/ae350" -I"../src/bsp/driver/include" -I"../src/bsp/lib" -I"../src/app" -I"../src/freertos/include" -I"../src/freertos/portable/GCC/RISC-V" -I"../src/freertos/portable/GCC/RISC-V/chip_specific_extensions/Andes_V5_RV32RV64" -I"../src/lwip/include" -I"../src/app/arch" -Og -mcmodel=medium -g3 -Wall -mcpu=a25 -ffunction-sections -fdata-sections -c -fmessage-length=0 -fno-builtin -fomit-frame-pointer -fno-strict-aliasing -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $(@:%.o=%.o)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


