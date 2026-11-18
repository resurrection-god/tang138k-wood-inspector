################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../.metadata/.plugins/org.eclipse.cdt.make.core/specs.c 

OBJS += \
./.metadata/.plugins/org.eclipse.cdt.make.core/specs.o 

C_DEPS += \
./.metadata/.plugins/org.eclipse.cdt.make.core/specs.d 


# Each subdirectory must supply rules for building sources it contributes
.metadata/.plugins/org.eclipse.cdt.make.core/%.o: ../.metadata/.plugins/org.eclipse.cdt.make.core/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Andes C Compiler'
	$(CROSS_COMPILE)gcc -DportasmHANDLE_INTERRUPT=mext_interrupt -DREF_TEST_CLK -I"../src/bsp/ae350" -I"../src/bsp/config" -I"../src/bsp/driver/ae350" -I"../src/bsp/driver/include" -I"../src/bsp/lib" -I"../src/app" -I"../src/freertos/include" -I"../src/freertos/portable/GCC/RISC-V" -I"../src/freertos/portable/GCC/RISC-V/chip_specific_extensions/Andes_V5_RV32RV64" -I"../src/lwip/include" -I"../src/app/arch" -Og -mcmodel=medium -g3 -Wall -mcpu=a25 -ffunction-sections -fdata-sections -c -fmessage-length=0 -fno-builtin -fomit-frame-pointer -fno-strict-aliasing -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $(@:%.o=%.o)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


