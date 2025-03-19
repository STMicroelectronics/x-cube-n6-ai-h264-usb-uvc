 /**
 ******************************************************************************
 * @file    app.h
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef APP_H
#define APP_H

#define VENC_WIDTH CAM_GetVencWidth()
#define VENC_HEIGHT CAM_GetVencHeight()
#define CAPTURE_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_ARGB8888
#define CAPTURE_BPP 4

/* Model Related Info */
#define NN_WIDTH 224
#define NN_HEIGHT 224
#define NN_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1
#define NN_BPP 3

void app_run(void);

#endif