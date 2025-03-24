C_SOURCES_GCC += Gcc/Src/console.c
C_SOURCES_GCC += Gcc/Src/freertos_libc.c
C_SOURCES_GCC += Gcc/Src/fast_memcpy.c
C_SOURCES_GCC += Gcc/Src/syscalls.c

ASM_SOURCES_GCC += $(FW_REL_DIR)/Drivers/CMSIS/Device/ST/STM32N6xx/Source/Templates/gcc/startup_stm32n657xx.s

C_SOURCES += $(C_SOURCES_GCC)
ASM_SOURCES += $(ASM_SOURCES_GCC)
