C_SOURCES_USBX += $(wildcard $(USBX_REL_DIR)/common/core/src/ux_system_*.c)
C_SOURCES_USBX += $(wildcard $(USBX_REL_DIR)/common/core/src/ux_utility_*.c)
C_SOURCES_USBX += $(wildcard $(USBX_REL_DIR)/common/core/src/ux_device_*.c)
C_SOURCES_USBX += $(wildcard $(USBX_REL_DIR)/common/usbx_stm32_device_controllers/ux_dcd_stm32_*.c)
C_SOURCES_USBX += $(wildcard $(USBX_REL_DIR)/common/usbx_device_classes/src/ux_device_class_video_*.c)
C_SOURCES_USBX += $(UVC_LIB_REL_DIR)/Src/usbx/uvcl_usbx.c

C_INCLUDES_USBX += -I$(USBX_REL_DIR)/common/core/inc
C_INCLUDES_USBX += -I$(USBX_REL_DIR)/ports/generic/inc
C_INCLUDES_USBX += -I$(USBX_REL_DIR)/common/usbx_device_classes/inc
C_INCLUDES_USBX += -I$(USBX_REL_DIR)/common/usbx_stm32_device_controllers
C_INCLUDES_USBX += -I$(UVC_LIB_REL_DIR)/Inc/usbx
C_INCLUDES_USBX += -I$(UVC_LIB_REL_DIR)/Src/usbx

C_DEFS_USBX += -DUVC_LIB_USE_USBX
C_DEFS_USBX += -DUX_INCLUDE_USER_DEFINE_FILE
C_DEFS_USBX += -DUSBL_PACKET_PER_MICRO_FRAME=3

ifeq ($(UVC_LIB_RTOS), NONE)
C_DEFS_USBX += -DUX_STANDALONE
C_DEFS_USBX += -DUVC_LIB_USE_SOF
else ifeq  ($(UVC_LIB_RTOS), THREADX)
# empty
else
$(error $(UVC_LIB_RTOS) is not supported) 
endif

C_SOURCES += $(C_SOURCES_USBX)
C_INCLUDES += $(C_INCLUDES_USBX)
C_DEFS += $(C_DEFS_USBX)
