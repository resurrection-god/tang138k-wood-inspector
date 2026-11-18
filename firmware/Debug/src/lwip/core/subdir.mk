################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/lwip/core/altcp.c \
../src/lwip/core/altcp_alloc.c \
../src/lwip/core/altcp_tcp.c \
../src/lwip/core/def.c \
../src/lwip/core/dns.c \
../src/lwip/core/inet_chksum.c \
../src/lwip/core/init.c \
../src/lwip/core/ip.c \
../src/lwip/core/mem.c \
../src/lwip/core/memp.c \
../src/lwip/core/netif.c \
../src/lwip/core/pbuf.c \
../src/lwip/core/raw.c \
../src/lwip/core/stats.c \
../src/lwip/core/sys.c \
../src/lwip/core/tcp.c \
../src/lwip/core/tcp_in.c \
../src/lwip/core/tcp_out.c \
../src/lwip/core/timeouts.c \
../src/lwip/core/udp.c 

OBJS += \
./src/lwip/core/altcp.o \
./src/lwip/core/altcp_alloc.o \
./src/lwip/core/altcp_tcp.o \
./src/lwip/core/def.o \
./src/lwip/core/dns.o \
./src/lwip/core/inet_chksum.o \
./src/lwip/core/init.o \
./src/lwip/core/ip.o \
./src/lwip/core/mem.o \
./src/lwip/core/memp.o \
./src/lwip/core/netif.o \
./src/lwip/core/pbuf.o \
./src/lwip/core/raw.o \
./src/lwip/core/stats.o \
./src/lwip/core/sys.o \
./src/lwip/core/tcp.o \
./src/lwip/core/tcp_in.o \
./src/lwip/core/tcp_out.o \
./src/lwip/core/timeouts.o \
./src/lwip/core/udp.o 

C_DEPS += \
./src/lwip/core/altcp.d \
./src/lwip/core/altcp_alloc.d \
./src/lwip/core/altcp_tcp.d \
./src/lwip/core/def.d \
./src/lwip/core/dns.d \
./src/lwip/core/inet_chksum.d \
./src/lwip/core/init.d \
./src/lwip/core/ip.d \
./src/lwip/core/mem.d \
./src/lwip/core/memp.d \
./src/lwip/core/netif.d \
./src/lwip/core/pbuf.d \
./src/lwip/core/raw.d \
./src/lwip/core/stats.d \
./src/lwip/core/sys.d \
./src/lwip/core/tcp.d \
./src/lwip/core/tcp_in.d \
./src/lwip/core/tcp_out.d \
./src/lwip/core/timeouts.d \
./src/lwip/core/udp.d 


# Each subdirectory must supply rules for building sources it contributes
src/lwip/core/%.o: ../src/lwip/core/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Andes C Compiler'
	$(CROSS_COMPILE)gcc -DportasmHANDLE_INTERRUPT=mext_interrupt -DREF_TEST_CLK -I"../src/bsp/ae350" -I"../src/bsp/config" -I"../src/bsp/driver/ae350" -I"../src/bsp/driver/include" -I"../src/bsp/lib" -I"../src/app" -I"../src/freertos/include" -I"../src/freertos/portable/GCC/RISC-V" -I"../src/freertos/portable/GCC/RISC-V/chip_specific_extensions/Andes_V5_RV32RV64" -I"../src/lwip/include" -I"../src/app/arch" -Og -mcmodel=medium -g3 -Wall -mcpu=a25 -ffunction-sections -fdata-sections -c -fmessage-length=0 -fno-builtin -fomit-frame-pointer -fno-strict-aliasing -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $(@:%.o=%.o)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


