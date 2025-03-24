######################################
# quiet mode
######################################
V = 0
ifeq ($(V), 0)
  quiet = quiet_
else
  quiet =
endif
quiet_CC  = @echo "  CC $@"; $(CC)
quiet_LD  = @echo "  LD $@"; $(CC)
quiet_AS  = @echo "  AS $@"; $(AS)
quiet_SZ  = @echo "  SZ $@"; $(SZ)
quiet_HEX  = @echo "  HEX $@"; $(HEX)
quiet_BIN  = @echo "  BIN $@"; $(BIN)

######################################
# helpers
######################################
rwildcard=$(foreach d,$(wildcard $(1:=/*)),$(call rwildcard,$d,$2) $(filter $(subst *,%,$2),$d))

######################################
# target
######################################
TARGET = Project
 # Supported Options: VD66GY; IMX335; VD55G1
SENSOR_LIST = IMX335 VD66GY VD55G1

MODEL_DIR = Model
BINARY_DIR = Binary

######################################
# building variables
######################################
OPT = -Os -g3

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES += Src/main.c
C_SOURCES += Src/app.c
C_SOURCES += Src/draw.c
C_SOURCES += Src/app_enc.c
C_SOURCES += Src/app_fuseprogramming.c
C_SOURCES += Src/stm32n6xx_it.c
C_SOURCES += Model/network.c
C_SOURCES += Src/app_cam.c
C_SOURCES += Src/freertos_bsp.c

# ASM sources
ASM_SOURCES =
ASM_SOURCES_S =

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
READELF = $(GCC_PATH)/$(PREFIX)readelf
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
CP = $(PREFIX)objcopy
READELF = $(PREFIX)readelf
endif
LD = $(CC)
HEX = $(CP) -O ihex
BIN = $(CP) -O binary

#######################################
# CFLAGS
#######################################
CPU = -mcpu=cortex-m55 -mcmse -mthumb
FPU = -mfpu=fpv5-d16 -mfloat-abi=hard

# mcu
MCU = $(CPU) $(FPU)

# others c flags
CFLAGS_OTHERS = -std=c11

# C defines
C_DEFS += -DSTM32N657xx
C_DEFS += -DUSE_FULL_ASSERT
C_DEFS += -DUSE_FULL_LL_DRIVER
C_DEFS += -DVECT_TAB_SRAM

C_DEFS += $(foreach SENSOR, $(SENSOR_LIST), -DUSE_$(SENSOR)_SENSOR)

# We only support single model
C_DEFS += -DTX_MAX_PARALLEL_NETWORKS=1

# C includes
# Patched files
C_INCLUDES += -IInc


ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fstack-usage -fdata-sections -ffunction-sections -fcyclomatic-complexity
CFLAGS = $(CFLAGS_OTHERS) $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fstack-usage -fdata-sections -ffunction-sections -fcyclomatic-complexity
# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = Gcc/STM32N657xx.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) $(LDFLAGS_OTHERS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
# Uncomment to enable %f formatted output
LDFLAGS_OTHERS += -u _printf_float
# Avoid 'build/Project.elf has a LOAD segment with RWX permissions' warning
LDFLAGS_OTHERS += -Wl,--no-warn-rwx-segments
LDFLAGS += -Wl,--print-memory-usage

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# Include mk files
#######################################
include mks/fw.mk
include mks/ai.mk
include mks/cmw.mk
include mks/freertos.mk
include mks/gcc.mk
include mks/iar.mk
include mks/venc.mk
UVC_LIB_REL_DIR := Lib/uvcl
UVC_LIB_USB_DEVICE_STACK := USBX
UVC_LIB_RTOS := FREERTOS
UVC_LIB_USE_DMA := YES
USBX_REL_DIR := $(FW_REL_DIR)/Middlewares/ST/usbx
include Lib/uvcl/uvc_lib.mk

#######################################
# build the application
#######################################
OBJECTS = $(addprefix $(BUILD_DIR)/, $(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES_S:.S=.o))


$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$($(quiet)CC)  -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$($(quiet)CC)  -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$($(quiet)AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).list: $(OBJECTS)
	$(file > $@, $(OBJECTS))

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile $(BUILD_DIR)/$(TARGET).list
	$($(quiet)LD) @$(BUILD_DIR)/$(TARGET).list $(LDFLAGS) -o $@
	$($(quiet)SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$($(quiet)HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$($(quiet)BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(call rwildcard,$(BUILD_DIR),*.d)
