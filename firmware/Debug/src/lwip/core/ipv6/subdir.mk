################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/lwip/core/ipv6/dhcp6.c \
../src/lwip/core/ipv6/ethip6.c \
../src/lwip/core/ipv6/icmp6.c \
../src/lwip/core/ipv6/inet6.c \
../src/lwip/core/ipv6/ip6.c \
../src/lwip/core/ipv6/ip6_addr.c \
../src/lwip/core/ipv6/ip6_frag.c \
../src/lwip/core/ipv6/mld6.c \
../src/lwip/core/ipv6/nd6.c 

OBJS += \
./src/lwip/core/ipv6/dhcp6.o \
./src/lwip/core/ipv6/ethip6.o \
./src/lwip/core/ipv6/icmp6.o \
./src/lwip/core/ipv6/inet6.o \
./src/lwip/core/ipv6/ip6.o \
./src/lwip/core/ipv6/ip6_addr.o \
./src/lwip/core/ipv6/ip6_frag.o \
./src/lwip/core/ipv6/mld6.o \
./src/lwip/core/ipv6/nd6.o 

C_DEPS += \
./src/lwip/core/ipv6/dhcp6.d \
./src/lwip/core/ipv6/ethip6.d \
./src/lwip/core/ipv6/icmp6.d \
./src/lwip/core/ipv6/inet6.d \
./src/lwip/core/ipv6/ip6.d \
./src/lwip/core/ipv6/ip6_addr.d \
./src/lwip/core/ipv6/ip6_frag.d \
./src/lwip/core/ipv6/mld6.d \
./src/lwip/core/ipv6/nd6.d 


# Each subdirectory must supply rules for building sources it contributes
src/lwip/core/ipv6/%.o: ../src/lwip/core/ipv6/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Andes C Compiler'
	$(CROSS_COMPILE)gcc -DportasmHANDLE_INTERRUPT=mext_interrupt -DREF_TEST_CLK -I"../src/bsp/ae350" -I"../src/bsp/config" -I"../src/bsp/driver/ae350" -I"../src/bsp/driver/include" -I"../src/bsp/lib" -I"../src/app" -I"../src/freertos/include" -I"../src/freertos/portable/GCC/RISC-V" -I"../src/freertos/portable/GCC/RISC-V/chip_specific_extensions/Andes_V5_RV32RV64" -I"../src/lwip/include" -I"../src/app/arch" -Og -mcmodel=medium -g3 -Wall -mcpu=a25 -ffunction-sections -fdata-sections -c -fmessage-length=0 -fno-builtin -fomit-frame-pointer -fno-strict-aliasing -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $(@:%.o=%.o)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


