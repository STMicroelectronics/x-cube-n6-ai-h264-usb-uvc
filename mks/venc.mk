VENC_REL_DIR := $(FW_REL_DIR)/Middlewares/Third_Party/VideoEncoder
EWL_REL_DIR := $(FW_REL_DIR)/Middlewares/ST/VideoEncoder_EWL

C_SOURCES_VENC += $(EWL_REL_DIR)/ewl_impl.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264EncApi.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264Init.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264SequenceParameterSet.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264PictureParameterSet.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264Slice.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264Denoise.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264PictureBuffer.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264RateControl.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264Cabac.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264PutBits.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264Sei.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264NalUnit.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264Mad.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/h264/H264CodeFrame.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/common/encasiccontroller.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/common/encasiccontroller_v2.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/common/encpreprocess.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/common/encswhwregisters.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/jpeg/EncJpeg.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/jpeg/EncJpegCodeFrame.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/jpeg/EncJpegInit.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/jpeg/EncJpegPutBits.c
C_SOURCES_VENC += $(VENC_REL_DIR)/source/jpeg/JpegEncApi.c

C_INCLUDES_VENC += -I$(EWL_REL_DIR)
C_INCLUDES_VENC += -I$(VENC_REL_DIR)/inc
C_INCLUDES_VENC += -I$(VENC_REL_DIR)/source/common

#C_DEFS_VENC += -DH264ENC_TRACE

C_SOURCES += $(C_SOURCES_VENC)
C_INCLUDES += $(C_INCLUDES_VENC)
C_DEFS += $(C_DEFS_VENC)
