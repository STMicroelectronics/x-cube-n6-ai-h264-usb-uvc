THREADX_REL_DIR := STM32Cube_FW_N6/Middlewares/ST
THREADX_CPU := cortex_m55

C_SOURCES_THREADX = $(wildcard $(THREADX_REL_DIR)/threadx/common/src/*.c)
C_SOURCES_THREADX_GCC += $(wildcard $(THREADX_REL_DIR)/threadx/ports/$(THREADX_CPU)/gnu/src/*.c)
C_SOURCES_THREADX_IAR += $(wildcard $(THREADX_REL_DIR)/threadx/ports/$(THREADX_CPU)/iar/src/*.c)
C_SOURCES_THREADX += $(THREADX_REL_DIR)/threadx/utility/execution_profile_kit/tx_execution_profile.c

ASM_SOURCES_THREADX_GCC += $(filter-out %/tx_initialize_low_level.S, $(wildcard $(THREADX_REL_DIR)/threadx/ports/$(THREADX_CPU)/gnu/src/*.S))
ASM_SOURCES_THREADX_IAR += $(filter-out %/tx_initialize_low_level.s %/tx_misra.s, $(wildcard $(THREADX_REL_DIR)/threadx/ports/$(THREADX_CPU)/iar/src/*.s))

C_INCLUDES_THREADX += -I$(THREADX_REL_DIR)/threadx/common/inc
C_INCLUDES_THREADX += -I$(THREADX_REL_DIR)/threadx/ports/$(THREADX_CPU)/gnu/inc
C_INCLUDES_THREADX += -I$(THREADX_REL_DIR)/threadx/utility/execution_profile_kit

C_DEFS_THREADX += -DTX_SINGLE_MODE_SECURE
C_DEFS_THREADX += -DFEAT_THREADX
C_DEFS_THREADX += -DTX_INCLUDE_USER_DEFINE_FILE
C_DEFS_THREADX += -DTX_EXECUTION_PROFILE_ENABLE

C_SOURCES += $(C_SOURCES_THREADX)
C_SOURCES_GCC += $(C_SOURCES_THREADX_GCC)
C_SOURCES_IAR += $(C_SOURCES_THREADX_IAR)
ASM_SOURCES_S_GCC += $(ASM_SOURCES_THREADX_GCC)
ASM_SOURCES_IAR += $(ASM_SOURCES_THREADX_IAR)
C_INCLUDES += $(C_INCLUDES_THREADX)
C_DEFS += $(C_DEFS_THREADX)
