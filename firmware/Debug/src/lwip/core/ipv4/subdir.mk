################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/lwip/core/ipv4/autoip.c \
../src/lwip/core/ipv4/dhcp.c \
../src/lwip/core/ipv4/etharp.c \
../src/lwip/core/ipv4/icmp.c \
../src/lwip/core/ipv4/igmp.c \
../src/lwip/core/ipv4/ip4.c \
../src/lwip/core/ipv4/ip4_addr.c \
../src/lwip/core/ipv4/ip4_frag.c 

OBJS += \
./src/lwip/core/ipv4/autoip.o \
./src/lwip/core/ipv4/dhcp.o \
./src/lwip/core/ipv4/etharp.o \
./src/lwip/core/ipv4/icmp.o \
./src/lwip/core/ipv4/igmp.o \
./src/lwip/core/ipv4/ip4.o \
./src/lwip/core/ipv4/ip4_addr.o \
./src/lwip/core/ipv4/ip4_frag.o 

C_DEPS += \
./src/lwip/core/ipv4/autoip.d \
./src/lwip/core/ipv4/dhcp.d \
./src/lwip/core/ipv4/etharp.d \
./src/lwip/core/ipv4/icmp.d \
./src/lwip/core/ipv4/igmp.d \
./src/lwip/core/ipv4/ip4.d \
./src/lwip/core/ipv4/ip4_addr.d \
./src/lwip/core/ipv4/ip4_frag.d 


# Each subdirectory must supply rules for building sources it contributes
src/lwip/core/ipv4/%.o: ../src/lwip/core/ipv4/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Andes C Compiler'
	$(CROSS_COMPILE)gcc -DportasmHANDLE_INTERRUPT=mext_interrupt -DREF_TEST_CLK -I"../src/bsp/ae350" -I"../src/bsp/config" -I"../src/bsp/driver/ae350" -I"../src/bsp/driver/include" -I"../src/bsp/lib" -I"../src/app" -I"../src/freertos/include" -I"../src/freertos/portable/GCC/RISC-V" -I"../src/freertos/portable/GCC/RISC-V/chip_specific_extensions/Andes_V5_RV32RV64" -I"../src/lwip/include" -I"../src/app/arch" -Og -mcmodel=medium -g3 -Wall -mcpu=a25 -ffunction-sections -fdata-sections -c -fmessage-length=0 -fno-builtin -fomit-frame-pointer -fno-strict-aliasing -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $(@:%.o=%.o)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


