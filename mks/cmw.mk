CMW_REL_DIR := Lib/Camera_Middleware
ISP_REL_DIR := $(CMW_REL_DIR)/ISP_Library

C_SOURCES_CMW += $(CMW_REL_DIR)/cmw_camera.c
C_SOURCES_CMW += $(CMW_REL_DIR)/cmw_utils.c
ifneq (,$(findstring VD55G1, $(SENSOR_LIST)))
C_SOURCES_CMW += $(CMW_REL_DIR)/sensors/cmw_vd55g1.c
C_SOURCES_CMW += $(CMW_REL_DIR)/sensors/vd55g1/vd55g1.c
endif
ifneq (,$(findstring IMX335, $(SENSOR_LIST)))
C_SOURCES_CMW += $(CMW_REL_DIR)/sensors/cmw_imx335.c
C_SOURCES_CMW += $(CMW_REL_DIR)/sensors/imx335/imx335.c
C_SOURCES_CMW += $(CMW_REL_DIR)/sensors/imx335/imx335_reg.c
endif
ifneq (,$(findstring VD66GY, $(SENSOR_LIST)))
C_SOURCES_CMW += $(CMW_REL_DIR)/sensors/cmw_vd66gy.c
C_SOURCES_CMW += $(CMW_REL_DIR)/sensors/vd6g/vd6g.c
endif
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_algo.c
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_cmd_parser.c
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_core.c
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_services.c
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_tool_com.c
C_INCLUDES_CMW += -I$(CMW_REL_DIR)
C_INCLUDES_CMW += -I$(CMW_REL_DIR)/sensors
C_INCLUDES_CMW += -I$(CMW_REL_DIR)/sensors/imx335
C_INCLUDES_CMW += -I$(CMW_REL_DIR)/sensors/vd55g1
C_INCLUDES_CMW += -I$(CMW_REL_DIR)/sensors/vd6g

C_INCLUDES_CMW += -I$(ISP_REL_DIR)/isp/Inc
C_INCLUDES_CMW += -I$(ISP_REL_DIR)/evision/Inc

C_SOURCES += $(C_SOURCES_CMW)
C_INCLUDES += $(C_INCLUDES_CMW)
C_DEFS += $(C_DEFS_CMW)
LIBS += -ln6-evision-awb_gcc -ln6-evision-st-ae_gcc
LIBDIR += -L$(ISP_REL_DIR)/evision/Lib
LIBS_IAR += $(ISP_REL_DIR)/evision/Lib/libn6-evision-awb_iar.a
LIBS_IAR += $(ISP_REL_DIR)/evision/Lib/libn6-evision-st-ae_iar.a
