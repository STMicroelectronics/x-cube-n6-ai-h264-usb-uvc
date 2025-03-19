# following makefile variables must be defined by caller
# UVC_LIB_REL_DIR : location of uvc_lib source code
# UVC_LIB_USB_DEVICE_STACK : USBX or STM32_USBD
# UVC_LIB_RTOS : either NONE, THREADX
# UVC_LIB_USE_DMA : either YES or NO. If YES you must have a section named .uncached_bss that is uncached
# USBX_REL_DIR : location of usbx source code. Only use if UVC_LIB_USB_DEVICE_STACK value is USBX
# STM32_USBD_REL_DIR : location of mw_usb_device source code. Only use if UVC_LIB_USB_DEVICE_STACK value is STM32_USBD

ifeq ($(UVC_LIB_USB_DEVICE_STACK), USBX)
include $(UVC_LIB_REL_DIR)/mks/uvc_lib_usbx.mk
else ifeq ($(UVC_LIB_USB_DEVICE_STACK), STM32_USBD)
include $(UVC_LIB_REL_DIR)/mks/uvc_lib_stm32_usbd.mk
else
$(error $(UVC_LIB_USB_DEVICE_STACK) is not supported)
endif

ifeq ($(UVC_LIB_USE_DMA), YES)
C_DEFS_UVC_LIB += -DUVC_LIB_USE_DMA
endif

C_SOURCES_UVC_LIB += $(UVC_LIB_REL_DIR)/Src/uvcl.c
C_SOURCES_UVC_LIB += $(UVC_LIB_REL_DIR)/Src/uvcl_desc.c

C_INCLUDES_UVC_LIB += -I$(UVC_LIB_REL_DIR)/Inc
C_INCLUDES_UVC_LIB += -I$(UVC_LIB_REL_DIR)/Src

C_SOURCES += $(C_SOURCES_UVC_LIB)
C_INCLUDES += $(C_INCLUDES_UVC_LIB)
C_DEFS += $(C_DEFS_UVC_LIB)
