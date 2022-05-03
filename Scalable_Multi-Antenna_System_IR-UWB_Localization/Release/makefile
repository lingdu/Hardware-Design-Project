################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

BUILD_ARTIFACT_NAME := Scalable_Multi-Antenna_System_IR-UWB_Localization
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Scalable_Multi-Antenna_System_IR-UWB_Localization.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Scalable_Multi-Antenna_System_IR-UWB_Localization.list \

OBJCOPY_BIN += \
Scalable_Multi-Antenna_System_IR-UWB_Localization.bin \


# All Target
all: main-build

# Main-build Target
main-build: Scalable_Multi-Antenna_System_IR-UWB_Localization.elf secondary-outputs

# Tool invocations
Scalable_Multi-Antenna_System_IR-UWB_Localization.elf: $(OBJS) $(USER_OBJS) D:\OneDrive\ -\ UGent\UGent\Burgerlijk_Ingenieur\Semester\ 2\HOP\Software\CubeIDE\Scalable_Multi-Antenna_System_IR-UWB_Localization\STM32L011F3PX_FLASH.ld
	arm-none-eabi-gcc -o "Scalable_Multi-Antenna_System_IR-UWB_Localization.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"D:\OneDrive - UGent\UGent\Burgerlijk_Ingenieur\Semester 2\HOP\Software\CubeIDE\Scalable_Multi-Antenna_System_IR-UWB_Localization\STM32L011F3PX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Scalable_Multi-Antenna_System_IR-UWB_Localization.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Scalable_Multi-Antenna_System_IR-UWB_Localization.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Scalable_Multi-Antenna_System_IR-UWB_Localization.list"
	@echo 'Finished building: $@'
	@echo ' '

Scalable_Multi-Antenna_System_IR-UWB_Localization.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Scalable_Multi-Antenna_System_IR-UWB_Localization.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets
