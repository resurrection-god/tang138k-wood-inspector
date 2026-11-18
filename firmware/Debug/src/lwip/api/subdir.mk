################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/lwip/api/api_lib.c \
../src/lwip/api/api_msg.c \
../src/lwip/api/err.c \
../src/lwip/api/if_api.c \
../src/lwip/api/netbuf.c \
../src/lwip/api/netdb.c \
../src/lwip/api/netifapi.c \
../src/lwip/api/sockets.c \
../src/lwip/api/tcpip.c 

OBJS += \
./src/lwip/api/api_lib.o \
./src/lwip/api/api_msg.o \
./src/lwip/api/err.o \
./src/lwip/api/if_api.o \
./src/lwip/api/netbuf.o \
./src/lwip/api/netdb.o \
./src/lwip/api/netifapi.o \
./src/lwip/api/sockets.o \
./src/lwip/api/tcpip.o 

C_DEPS += \
./src/lwip/api/api_lib.d \
./src/lwip/api/api_msg.d \
./src/lwip/api/err.d \
./src/lwip/api/if_api.d \
./src/lwip/api/netbuf.d \
./src/lwip/api/netdb.d \
./src/lwip/api/netifapi.d \
./src/lwip/api/sockets.d \
./src/lwip/api/tcpip.d 


# Each subdirectory must supply rules for building sources it contributes
src/lwip/api/%.o: ../src/lwip/api/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Andes C Compiler'
	$(CROSS_COMPILE)gcc -DportasmHANDLE_INTERRUPT=mext_interrupt -DREF_TEST_CLK -I"../src/bsp/ae350" -I"../src/bsp/config" -I"../src/bsp/driver/ae350" -I"../src/bsp/driver/include" -I"../src/bsp/lib" -I"../src/app" -I"../src/freertos/include" -I"../src/freertos/portable/GCC/RISC-V" -I"../src/freertos/portable/GCC/RISC-V/chip_specific_extensions/Andes_V5_RV32RV64" -I"../src/lwip/include" -I"../src/app/arch" -Og -mcmodel=medium -g3 -Wall -mcpu=a25 -ffunction-sections -fdata-sections -c -fmessage-length=0 -fno-builtin -fomit-frame-pointer -fno-strict-aliasing -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $(@:%.o=%.o)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


