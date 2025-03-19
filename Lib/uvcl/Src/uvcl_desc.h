/**
 ******************************************************************************
 * @file    uvcl_desc.h
 * @author  MDG Application Team
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

#ifndef _UVCL_DESC_H_
#define _UVCL_DESC_H_

#include <stdint.h>

typedef struct {
  int is_hs;
  int width;
  int height;
  int fps;
  int payload_type;
  uint32_t dwMaxVideoFrameSize;
} UVCL_DescConf;

int UVCL_get_device_desc(void *p_dst, int dst_len, int idx_manufacturer, int idx_product, int idx_serial);
int UVCL_get_device_qualifier_desc(void *p_dst, int dst_len);
int UVCL_get_lang_string_desc(void *p_dst, int dst_len);
int UVCL_get_manufacturer_string_desc(void *p_dst, int dst_len);
int UVCL_get_product_string_desc(void *p_dst, int dst_len);
int UVCL_get_serial_string_desc(void *p_dst, int dst_len);
int UVCL_get_configuration_desc(void *p_dst, int dst_len, UVCL_DescConf *p_conf);

#endif
