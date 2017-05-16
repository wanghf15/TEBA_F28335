################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv5/tools/compiler/c2000_6.1.3

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./src/F28335A_main.obj" \
"./src/DSP2833x_Xintf.obj" \
"./src/DSP2833x_SysInit.obj" \
"./src/DSP2833x_Spi.obj" \
"./src/DSP2833x_Sci.obj" \
"./src/DSP2833x_PieVect.obj" \
"./src/DSP2833x_PieCtrl.obj" \
"./src/DSP2833x_I2C.obj" \
"./src/DSP2833x_Gpio.obj" \
"./src/DSP2833x_GlobalVariableDefs.obj" \
"./src/DSP2833x_DefaultIsr.obj" \
"./src/DSP2833x_CpuTimers.obj" \
"./src/DSP2833x_CodeStartBranch.obj" \
"./src/DSP2833x_ADC_cal.obj" \
-l"libc.a" \
"../28335_FLASH_lnk.cmd" \
"../DSP2833x_Headers_nonBIOS.cmd" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include src/subdir_vars.mk
-include subdir_rules.mk
-include src/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: WX10KV160904_N12.out

# Tool invocations
WX10KV160904_N12.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv5/tools/compiler/c2000_6.1.3/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --define=FLASH --diag_warning=225 --display_error_number --diag_wrap=off -z --stack_size=0x300 -m"WX10KV160904_N12.map" --warn_sections -i"C:/ti/ccsv5/tools/compiler/c2000_6.1.3/lib" -i"C:/ti/ccsv5/tools/compiler/c2000_6.1.3/include" --reread_libs --display_error_number --diag_wrap=off --xml_link_info="WX10KV160904_N12_linkInfo.xml" --rom_model -o "WX10KV160904_N12.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C2000_EXECUTABLE_OUTPUTS__QUOTED) "WX10KV160904_N12.out"
	-$(RM) "src\DSP2833x_CpuTimers.pp" "src\DSP2833x_DefaultIsr.pp" "src\DSP2833x_GlobalVariableDefs.pp" "src\DSP2833x_Gpio.pp" "src\DSP2833x_I2C.pp" "src\DSP2833x_PieCtrl.pp" "src\DSP2833x_PieVect.pp" "src\DSP2833x_Sci.pp" "src\DSP2833x_Spi.pp" "src\DSP2833x_SysInit.pp" "src\DSP2833x_Xintf.pp" "src\F28335A_main.pp" 
	-$(RM) "src\DSP2833x_ADC_cal.obj" "src\DSP2833x_CodeStartBranch.obj" "src\DSP2833x_CpuTimers.obj" "src\DSP2833x_DefaultIsr.obj" "src\DSP2833x_GlobalVariableDefs.obj" "src\DSP2833x_Gpio.obj" "src\DSP2833x_I2C.obj" "src\DSP2833x_PieCtrl.obj" "src\DSP2833x_PieVect.obj" "src\DSP2833x_Sci.obj" "src\DSP2833x_Spi.obj" "src\DSP2833x_SysInit.obj" "src\DSP2833x_Xintf.obj" "src\F28335A_main.obj" 
	-$(RM) "src\DSP2833x_ADC_cal.pp" "src\DSP2833x_CodeStartBranch.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
