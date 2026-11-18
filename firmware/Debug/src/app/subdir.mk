################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/app/board.c \
../src/app/cam_trig.c \
../src/app/camera.c \
../src/app/console.c \
../src/app/ethernet.c \
../src/app/frame_udp.c \
../src/app/gev_xml.c \
../src/app/gvcp.c \
../src/app/gvsp.c \
../src/app/gvsp_gate.c \
../src/app/hik_client.c \
../src/app/i2c_bus.c \
../src/app/laser.c \
../src/app/lcd_hud.c \
../src/app/mailbox.c \
../src/app/main.c \
../src/app/mm_table.c \
../src/app/net_config.c \
../src/app/nvstore.c \
../src/app/ota.c \
../src/app/settings.c \
../src/app/spi_flash.c \
../src/app/sync.c \
../src/app/video.c \
../src/app/vmsp_protocol.c \
../src/app/vmsp_service.c \
../src/app/watchdog.c \
../src/app/work_mode.c 

OBJS += \
./src/app/board.o \
./src/app/cam_trig.o \
./src/app/camera.o \
./src/app/console.o \
./src/app/ethernet.o \
./src/app/frame_udp.o \
./src/app/gev_xml.o \
./src/app/gvcp.o \
./src/app/gvsp.o \
./src/app/gvsp_gate.o \
./src/app/hik_client.o \
./src/app/i2c_bus.o \
./src/app/laser.o \
./src/app/lcd_hud.o \
./src/app/mailbox.o \
./src/app/main.o \
./src/app/mm_table.o \
./src/app/net_config.o \
./src/app/nvstore.o \
./src/app/ota.o \
./src/app/settings.o \
./src/app/spi_flash.o \
./src/app/sync.o \
./src/app/video.o \
./src/app/vmsp_protocol.o \
./src/app/vmsp_service.o \
./src/app/watchdog.o \
./src/app/work_mode.o 

C_DEPS += \
./src/app/board.d \
./src/app/cam_trig.d \
./src/app/camera.d \
./src/app/console.d \
./src/app/ethernet.d \
./src/app/frame_udp.d \
./src/app/gev_xml.d \
./src/app/gvcp.d \
./src/app/gvsp.d \
./src/app/gvsp_gate.d \
./src/app/hik_client.d \
./src/app/i2c_bus.d \
./src/app/laser.d \
./src/app/lcd_hud.d \
./src/app/mailbox.d \
./src/app/main.d \
./src/app/mm_table.d \
./src/app/net_config.d \
./src/app/nvstore.d \
./src/app/ota.d \
./src/app/settings.d \
./src/app/spi_flash.d \
./src/app/sync.d \
./src/app/video.d \
./src/app/vmsp_protocol.d \
./src/app/vmsp_service.d \
./src/app/watchdog.d \
./src/app/work_mode.d 


# Each subdirectory must supply rules for building sources it contributes
src/app/%.o: ../src/app/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Andes C Compiler'
	$(CROSS_COMPILE)gcc -DportasmHANDLE_INTERRUPT=mext_interrupt -DREF_TEST_CLK -I"../src/bsp/ae350" -I"../src/bsp/config" -I"../src/bsp/driver/ae350" -I"../src/bsp/driver/include" -I"../src/bsp/lib" -I"../src/app" -I"../src/freertos/include" -I"../src/freertos/portable/GCC/RISC-V" -I"../src/freertos/portable/GCC/RISC-V/chip_specific_extensions/Andes_V5_RV32RV64" -I"../src/lwip/include" -I"../src/app/arch" -Og -mcmodel=medium -g3 -Wall -mcpu=a25 -ffunction-sections -fdata-sections -c -fmessage-length=0 -fno-builtin -fomit-frame-pointer -fno-strict-aliasing -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $(@:%.o=%.o)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


