################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
FaTFS/option/cc932.obj: ../FaTFS/option/cc932.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.9.0.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --opt_for_speed=2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.9.0.LTS/include" --include_path="C:/Users/karlo/OneDrive - alum.us.es/Documentos/ARUS/ADQ-ART-17/ADQ_1.0/FaTFS" --include_path="C:/ti/TivaWare_C_Series-2.1.3.156" --define=ccs="ccs" --define=PART_TM4C123GH6PM -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="FaTFS/option/cc932.d" --obj_directory="FaTFS/option" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


