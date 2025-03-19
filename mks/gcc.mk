C_SOURCES_GCC += Gcc/Src/console.c
C_SOURCES_GCC += Gcc/Src/threadx_libc.c
C_SOURCES_GCC += Gcc/Src/fast_memcpy.c
C_SOURCES_GCC += Gcc/Src/syscalls.c

ASM_SOURCES_GCC += $(FW_REL_DIR)/Drivers/CMSIS/Device/ST/STM32N6xx/Source/Templates/gcc/startup_stm32n657xx.s

ASM_SOURCES_S_GCC += Gcc/Src/tx_initialize_low_level.S

C_SOURCES += $(C_SOURCES_GCC)
ASM_SOURCES += $(ASM_SOURCES_GCC)
ASM_SOURCES_S += $(ASM_SOURCES_S_GCC)
