/**
 ******************************************************************************
 * @file    uvcl_desc.c
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

#include "uvcl_desc.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "stm32n6xx_hal.h"

#include "uvcl.h"
#include "uvcl_internal.h"
#include "uvcl_codes.h"
#include "uvcl_desc_internal.h"

#define container_of(ptr, type, member) (type *) ((unsigned char *)ptr - offsetof(type,member))
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))

#define DEV_MANUFACTURER_STRING      "STMicroelectronics"
#define DEV_PRODUCT_STRING           "STM32 uvc"

static void gen_serial_int_to_ascii(uint32_t value, char *p_buf, uint8_t len)
{
  uint8_t idx = 0U;

  for (idx = 0U ; idx < len ; idx ++)
  {
    if (((value >> 28)) < 0xAU)
      p_buf[idx] = (value >> 28) + '0';
    else
      p_buf[idx] = (value >> 28) + 'A' - 10U;
    value = value << 4;
  }
}

static void gen_serial(char serial[13])
{
  uint32_t deviceserial0;
  uint32_t deviceserial1;
  uint32_t deviceserial2;

  deviceserial0 = HAL_GetUIDw0();
  deviceserial1 = HAL_GetUIDw1();
  deviceserial2 = HAL_GetUIDw2();
  deviceserial0 += deviceserial2;

  gen_serial_int_to_ascii(deviceserial0, &serial[0], 8);
  gen_serial_int_to_ascii(deviceserial1, &serial[8], 4);
  serial[12] = '\0';
}

static int UVCL_get_string_desc(void *p_dst, int dst_len, char *name)
{
  int desc_len = 2 + strlen(name) * 2;
  uint8_t *dst = p_dst;

  if (dst_len < desc_len)
    return -1;

  *dst++ = desc_len; /* bLength = desc_len */
  *dst++ = 0x03; /* string descriptor */

  while (*name) {
    *dst++ = *name++;
    *dst++ = 0;
  }

  return dst_len;
}

static int get_children_len(struct uvc_desc_head *child)
{
  int total_len = 0;

  while (child) {
    total_len += child->bLength + get_children_len(child->children);
    child = child->next_child;
  }

  return total_len;
}

static void append_as_child(struct uvc_desc_head *parent, struct uvc_desc_head *child)
{
  struct uvc_desc_head *prev_child_head = parent->children;

  parent->children = child;
  child->next_child = prev_child_head;
}

static int gen_default_desc(struct uvc_desc_head *head, uint8_t *p_dst, int dst_len)
{
  if (head->bLength > dst_len)
    return -1;

  memcpy(p_dst, head->raw, head->bLength);
  if (head->next) {
    assert(head->next->gen);
    return head->next->gen(head->next, p_dst + head->bLength, dst_len - head->bLength);
  } else {
    return dst_len - head->bLength;
  }
}

static void update(struct uvc_desc_head *head)
{
  while (head) {
    if (head->update)
      head->update(head);
    head = head->next;
  }
}

static int generate(struct uvc_desc_head *head, uint8_t *p_dst, int dst_len)
{
  int ret;

  ret = head->gen(head, p_dst, dst_len);

  return ret < 0 ? ret : dst_len - ret;
}

static void build_dev_desc(struct uvc_dev_desc *desc, struct uvc_desc_head *next, UVCL_DescConf *p_conf,
                           int idx_manufacturer, int idx_product, int idx_serial)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = 0x01;
  desc->raw.bcdUSB = 0x0200;
  desc->raw.bDeviceClass = 0x00;
  desc->raw.bDeviceSubClass = 0x00;
  desc->raw.bDeviceProtocol = 0x00;
  desc->raw.bMaxPacketSize = 64;
  desc->raw.idVendor = 0x0483;
  desc->raw.idProduct = 0x5780;
  desc->raw.bcdDevice = 0x0100;
  desc->raw.iManufacturer = idx_manufacturer;
  desc->raw.iProduct = idx_product;
  desc->raw.iSerialNumber = idx_serial;
  desc->raw.bNumConfigurations = 1;
}

static void update_uvc_conf_desc(struct uvc_desc_head *head)
{
  struct uvc_conf_desc *desc = container_of(head, struct uvc_conf_desc, head);

  desc->raw.wTotalLength += get_children_len(desc->head.children);
}

static void build_uvc_conf_desc(struct uvc_conf_desc *desc, struct uvc_desc_head *next, UVCL_DescConf *p_conf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.update = update_uvc_conf_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = 0x02;
  desc->raw.wTotalLength = desc->raw.bLength;
  desc->raw.bNumInterfaces = 2;
  desc->raw.bConfigurationValue = 1;
  desc->raw.iConfiguration = 0;
  desc->raw.bmAttributes = 0xc0; /* self powered */
  desc->raw.bMaxPower = 50; /* 100 ma */
}

static void build_uvc_iad_desc(struct uvc_iad_desc *desc, struct uvc_desc_head *next, struct uvc_desc_head *parent,
                               UVCL_DescConf *p_conf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = 0x0b;
  desc->raw.bFirstInterface = 0;
  desc->raw.bInterfaceCount = 2;
  desc->raw.bFunctionClass = CC_VIDEO;
  desc->raw.bFunctionSubClass = SC_VIDEO_INTERFACE_COLLECTION;
  desc->raw.bFunctionProtocol = PC_PROTOCOL_UNDEFINED;
  desc->raw.iFunction = 0;

  append_as_child(parent, &desc->head);
}

static void build_uvc_std_vc_desc(struct uvc_std_vc_desc *desc, struct uvc_desc_head *next, struct uvc_desc_head *parent,
                                  UVCL_DescConf *p_conf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = 0x04;
  desc->raw.bInterfaceNumber = 0;
  desc->raw.bAlternateSetting = 0;
  desc->raw.bNumEndpoints = 0;
  desc->raw.bInterfaceClass = CC_VIDEO;
  desc->raw.bInterfaceSubClass = SC_VIDEOCONTROL;
  desc->raw.bInterfaceProtocol = PC_PROTOCOL_15;
  desc->raw.iInterface = 0;

  append_as_child(parent, &desc->head);
}

static void update_uvc_class_vc_desc(struct uvc_desc_head *head)
{
  struct uvc_class_vc_desc *desc = container_of(head, struct uvc_class_vc_desc, head);

  desc->raw.wTotalLength += get_children_len(desc->head.children);
}

static void build_uvc_class_vc_desc(struct uvc_class_vc_desc *desc, struct uvc_desc_head *next,
                                    struct uvc_desc_head *parent, UVCL_DescConf *p_conf)
{
  const int vs_nb = 1;

  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.update = update_uvc_class_vc_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = VC_HEADER;
  desc->raw.bcdUVC = 0x0150;
  desc->raw.wTotalLength = 12 + vs_nb;;
  desc->raw.dwClockFrequency =48000000;
  desc->raw.bInCollection = vs_nb;
  for (int i = 0; i < vs_nb; i++)
    desc->raw.baInterfaceNr[i] = i + 1;

  append_as_child(parent, &desc->head);
}

static void build_uvc_camera_terminal_desc(struct uvc_camera_terminal_desc *desc, struct uvc_desc_head *next,
                                           struct uvc_desc_head *parent, UVCL_DescConf *p_conf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = VC_INPUT_TERMINAL;
  desc->raw.bTerminalID = 1;
  desc->raw.wTerminalType = ITT_CAMERA;
  desc->raw.bAssocTerminal = 0;
  desc->raw.iTerminal = 0;
  desc->raw.wObjectiveFocalLengthMin = 0;
  desc->raw.wObjectiveFocalLengthMax = 0;
  desc->raw.wOcularFocalLength = 0;
  desc->raw.bControlSize = 3;
  for (int i = 0; i < 3; i++)
    desc->raw.bmControls[i] = 0;

  append_as_child(parent, &desc->head);
}

static void build_uvc_output_term_desc(struct uvc_output_term_desc *desc, struct uvc_desc_head *next,
                                       struct uvc_desc_head *parent, UVCL_DescConf *p_conf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = VC_OUTPUT_TERMINAL;
  desc->raw.bTerminalID = 2;
  desc->raw.wTerminalType = TT_STREAMING;
  desc->raw.bAssocTerminal = 0;
  desc->raw.bSourceID = 1;
  desc->raw.iTerminal = 0;

  append_as_child(parent, &desc->head);
}

static void build_uvc_std_vs_desc(struct uvc_std_vs_desc *desc, struct uvc_desc_head *next, struct uvc_desc_head *parent,
                                  UVCL_DescConf *p_conf, int alt_nb, int ep_nb)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = 0x04;
  desc->raw.bInterfaceNumber = 1;
  desc->raw.bAlternateSetting = alt_nb;
  desc->raw.bNumEndpoints = ep_nb;
  desc->raw.bInterfaceClass = CC_VIDEO;
  desc->raw.bInterfaceSubClass = SC_VIDEOSTREAMING;
  desc->raw.bInterfaceProtocol = PC_PROTOCOL_UNDEFINED;
  desc->raw.iInterface = 0;

  append_as_child(parent, &desc->head);
}

static void update_uvc_vs_input_desc(struct uvc_desc_head *head)
{
  struct uvc_vs_input_desc *desc = container_of(head, struct uvc_vs_input_desc, head);

  desc->raw.wTotalLength += get_children_len(desc->head.children);
}

static void build_uvc_vs_input_desc(struct uvc_vs_input_desc *desc, struct uvc_desc_head *next,
                                    struct uvc_desc_head *parent, UVCL_DescConf *p_conf, int format_nb)
{
  assert(format_nb <= ARRAY_LEN(desc->raw.bmaControls));

  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.update = update_uvc_vs_input_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = VS_INPUT_HEADER;
  desc->raw.bNumFormats = format_nb;
  desc->raw.wTotalLength = desc->head.bLength;
  desc->raw.bEndpointAddress = 0x81;
  desc->raw.bmInfo = 0;
  desc->raw.bTerminalLink = 2;
  desc->raw.bStillCaptureMethod = 0;
  desc->raw.bTriggerSupport = 0;
  desc->raw.bTriggerUsage = 0;
  desc->raw.bControlSize = 1;
  desc->raw.bmaControls[0] = 0;

  append_as_child(parent, &desc->head);
}

static void build_uvc_yuv422_fmt_desc(struct uvc_yuv422_fmt_desc *desc, struct uvc_desc_head *next,
                                       struct uvc_desc_head *parent, UVCL_DescConf *p_conf)
{
  const uint8_t YUV422_guid[] = {'Y', 'U', 'Y', '2', 0x00, 0x00, 0x10, 0x00,
                 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71};
                 int i;

  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = VS_FORMAT_UNCOMPRESSED;
  desc->raw.bFormatIndex = 1;
  desc->raw.bNumFrameDescriptors = 1;
  for (i = 0; i < 16; i++)
    desc->raw.guidFormat[i] = YUV422_guid[i];
  desc->raw.bBitsPerPixel = 16;
  desc->raw.bDefaultFrameIndex = 1;
  desc->raw.bAspectRatioX = 0;
  desc->raw.bAspectRatioY = 0;
  desc->raw.bmInterlaceFlags = 0;
  desc->raw.bCopyProtect = 0;

  append_as_child(parent, &desc->head);
}

static void build_uvc_yuv422_frame_desc(struct uvc_yuv422_frame_desc *desc, struct uvc_desc_head *next,
                                         struct uvc_desc_head *parent, UVCL_DescConf *p_conf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = VS_FRAME_UNCOMPRESSED;
  desc->raw.bFrameIndex = 1;
  desc->raw.bmCapabilities = 2;
  desc->raw.wWidth = p_conf->width;
  desc->raw.wHeight = p_conf->height;
  desc->raw.dwMinBitRate = desc->raw.wWidth * desc->raw.wHeight * p_conf->fps * 16;
  desc->raw.dwMaxBitRate = desc->raw.dwMinBitRate;
  desc->raw.dwDefaultFrameInterval = 10000000 / p_conf->fps;
  desc->raw.bFrameIntervalType = 1;
  desc->raw.dwFrameInterval[0] = 10000000 / p_conf->fps;

  append_as_child(parent, &desc->head);
}

static void build_uvc_color_desc(struct uvc_color_desc *desc, struct uvc_desc_head *next, struct uvc_desc_head *parent,
                                 UVCL_DescConf *p_conf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = VS_COLORFORMAT;
  desc->raw.bColorPrimaries = 1;
  desc->raw.bTransferCharacteristics = 1;
  desc->raw.bMatrixCoefficients = 4;

  append_as_child(parent, &desc->head);
}

static void build_uvc_vs_ep_desc(struct uvc_vs_ep_desc *desc, struct uvc_desc_head *next, struct uvc_desc_head *parent,
                                 UVCL_DescConf *p_conf, int wMaxPacketSize)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = 5;
  desc->raw.bEndpointAddress = 0x81;
  desc->raw.bmAttributes = 0x05;
  desc->raw.wMaxPacketSize = wMaxPacketSize;
  desc->raw.bInterval = 1;

  append_as_child(parent, &desc->head);
}

static void build_xx_yuv422_conf_desc(struct uvc_yuv422_conf_desc *desc, UVCL_DescConf *p_conf, int wMaxPacketSize)
{
  build_uvc_conf_desc(&desc->conf_desc, &desc->iad_desc.head, p_conf);
    build_uvc_iad_desc(&desc->iad_desc, &desc->std_vc_desc.head, &desc->conf_desc.head, p_conf);
      build_uvc_std_vc_desc(&desc->std_vc_desc, &desc->class_vc_desc.head, &desc->iad_desc.head, p_conf);
        build_uvc_class_vc_desc(&desc->class_vc_desc, &desc->cam_desc.head, &desc->std_vc_desc.head, p_conf);
          build_uvc_camera_terminal_desc(&desc->cam_desc, &desc->tt_desc.head, &desc->class_vc_desc.head, p_conf);
          build_uvc_output_term_desc(&desc->tt_desc, &desc->std_vs_alt0_desc.head, &desc->class_vc_desc.head, p_conf);
      build_uvc_std_vs_desc(&desc->std_vs_alt0_desc, &desc->vs_input_desc.head, &desc->iad_desc.head, p_conf, 0, 0);
        build_uvc_vs_input_desc(&desc->vs_input_desc, &desc->fb_fmt_desc.head, &desc->std_vs_alt0_desc.head, p_conf, 1);
          build_uvc_yuv422_fmt_desc(&desc->fb_fmt_desc, &desc->fb_frame_desc.head, &desc->vs_input_desc.head, p_conf);
            build_uvc_yuv422_frame_desc(&desc->fb_frame_desc, &desc->color_desc.head, &desc->fb_fmt_desc.head, p_conf);
          build_uvc_color_desc(&desc->color_desc, &desc->std_vs_alt1_desc.head, &desc->fb_fmt_desc.head, p_conf);
      build_uvc_std_vs_desc(&desc->std_vs_alt1_desc, &desc->ep_desc.head, &desc->iad_desc.head, p_conf, 1, 1);
        build_uvc_vs_ep_desc(&desc->ep_desc, NULL, &desc->std_vs_alt1_desc.head, p_conf, wMaxPacketSize);
}

static int UVCL_build_hs_yuv422_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  struct uvc_yuv422_conf_desc desc = { 0 };
  int wMaxPacketSize = 1024 | ((USBL_PACKET_PER_MICRO_FRAME - 1) << 11);

  assert(USBL_PACKET_PER_MICRO_FRAME >= 1 && USBL_PACKET_PER_MICRO_FRAME <= 3);

  build_xx_yuv422_conf_desc(&desc, p_conf, wMaxPacketSize);
  update(&desc.conf_desc.head);
  return generate(&desc.conf_desc.head, p_dst, dst_len);
}

static int UVCL_build_fs_yuv422_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  struct uvc_yuv422_conf_desc desc = { 0 };

  build_xx_yuv422_conf_desc(&desc, p_conf, 1023);
  update(&desc.conf_desc.head);
  return generate(&desc.conf_desc.head, p_dst, dst_len);
}

static int UVCL_build_yuv422_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  if (p_conf->is_hs)
    return UVCL_build_hs_yuv422_configuration_desc(p_dst, dst_len, p_conf);
  else
    return UVCL_build_fs_yuv422_configuration_desc(p_dst, dst_len, p_conf);
}

static void build_uvc_jpeg_fmt_desc(struct uvc_jpeg_fmt_desc *desc, struct uvc_desc_head *next,
                                    struct uvc_desc_head *parent, UVCL_DescConf *p_conf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = VS_FORMAT_MJPEG;
  desc->raw.bFormatIndex = 1;
  desc->raw.bNumFrameDescriptors = 1;
  desc->raw.bmFlags = 1;
  desc->raw.bDefaultFrameIndex = 1;
  desc->raw.bAspectRatioX = 0;
  desc->raw.bAspectRatioY = 0;
  desc->raw.bmInterlaceFlags = 0;
  desc->raw.bCopyProtect = 0;

  append_as_child(parent, &desc->head);
}

static void build_uvc_jpeg_frame_desc(struct uvc_jpeg_frame_desc *desc, struct uvc_desc_head *next,
                                      struct uvc_desc_head *parent, UVCL_DescConf *p_conf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);;

  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = VS_FRAME_MJPEG;
  desc->raw.bFrameIndex = 1;
  desc->raw.bmCapabilities = 2;
  desc->raw.wWidth = p_conf->width;
  desc->raw.wHeight = p_conf->height;
  desc->raw.dwMinBitRate = p_conf->dwMaxVideoFrameSize * p_conf->fps * 8;
  desc->raw.dwMaxBitRate = desc->raw.dwMinBitRate;
  desc->raw.dwMaxVideoFrameBufferSize = p_conf->dwMaxVideoFrameSize;
  desc->raw.dwDefaultFrameInterval = 10000000 / p_conf->fps;
  desc->raw.bFrameIntervalType = 1;
  desc->raw.dwFrameInterval[0] = 10000000 / p_conf->fps;

  append_as_child(parent, &desc->head);
}

static void build_xx_jpeg_conf_desc(struct uvc_jpeg_conf_desc *desc, UVCL_DescConf *p_conf, int wMaxPacketSize)
{
  build_uvc_conf_desc(&desc->conf_desc, &desc->iad_desc.head, p_conf);
    build_uvc_iad_desc(&desc->iad_desc, &desc->std_vc_desc.head, &desc->conf_desc.head, p_conf);
      build_uvc_std_vc_desc(&desc->std_vc_desc, &desc->class_vc_desc.head, &desc->iad_desc.head, p_conf);
        build_uvc_class_vc_desc(&desc->class_vc_desc, &desc->cam_desc.head, &desc->std_vc_desc.head, p_conf);
          build_uvc_camera_terminal_desc(&desc->cam_desc, &desc->tt_desc.head, &desc->class_vc_desc.head, p_conf);
          build_uvc_output_term_desc(&desc->tt_desc, &desc->std_vs_alt0_desc.head, &desc->class_vc_desc.head, p_conf);
      build_uvc_std_vs_desc(&desc->std_vs_alt0_desc, &desc->vs_input_desc.head, &desc->iad_desc.head, p_conf, 0, 0);
        build_uvc_vs_input_desc(&desc->vs_input_desc, &desc->fb_fmt_desc.head, &desc->std_vs_alt0_desc.head, p_conf, 1);
          build_uvc_jpeg_fmt_desc(&desc->fb_fmt_desc, &desc->fb_frame_desc.head, &desc->vs_input_desc.head, p_conf);
            build_uvc_jpeg_frame_desc(&desc->fb_frame_desc, &desc->color_desc.head, &desc->fb_fmt_desc.head, p_conf);
          build_uvc_color_desc(&desc->color_desc, &desc->std_vs_alt1_desc.head, &desc->fb_fmt_desc.head, p_conf);
      build_uvc_std_vs_desc(&desc->std_vs_alt1_desc, &desc->ep_desc.head, &desc->iad_desc.head, p_conf, 1, 1);
        build_uvc_vs_ep_desc(&desc->ep_desc, NULL, &desc->std_vs_alt1_desc.head, p_conf, wMaxPacketSize);
}

static int UVCL_build_hs_jpeg_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  struct uvc_jpeg_conf_desc desc = { 0 };
  int wMaxPacketSize = 1024 | ((USBL_PACKET_PER_MICRO_FRAME - 1) << 11);

  assert(USBL_PACKET_PER_MICRO_FRAME >= 1 && USBL_PACKET_PER_MICRO_FRAME <= 3);

  build_xx_jpeg_conf_desc(&desc, p_conf, wMaxPacketSize);
  update(&desc.conf_desc.head);
  return generate(&desc.conf_desc.head, p_dst, dst_len);
}

static int UVCL_build_fs_jpeg_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  struct uvc_jpeg_conf_desc desc = { 0 };

  build_xx_jpeg_conf_desc(&desc, p_conf, 1023);
  update(&desc.conf_desc.head);
  return generate(&desc.conf_desc.head, p_dst, dst_len);
}

static int UVCL_build_jpeg_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  if (p_conf->is_hs)
    return UVCL_build_hs_jpeg_configuration_desc(p_dst, dst_len, p_conf);
  else
    return UVCL_build_fs_jpeg_configuration_desc(p_dst, dst_len, p_conf);
}

static void build_uvc_fb_rgb565_fmt_desc(struct uvc_fb_fmt_desc *desc, UVCL_DescConf *p_conf)
{
  const uint8_t fb_rgb565_guid[] = {'R', 'G', 'B', 'P', 0x00, 0x00, 0x10, 0x00,
                 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71};
  int i;

  for (i = 0; i < 16; i++)
    desc->raw.guidFormat[i] = fb_rgb565_guid[i];
  desc->raw.bBitsPerPixel = 16;
  desc->raw.bVariableSize = 0;
}

static void build_uvc_fb_grey_fmt_desc(struct uvc_fb_fmt_desc *desc, UVCL_DescConf *p_conf)
{
  const uint8_t fb_grey_guid[] = {'Y', '8', ' ', ' ', 0x00, 0x00, 0x10, 0x00,
                 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71};
  int i;

  for (i = 0; i < 16; i++)
    desc->raw.guidFormat[i] = fb_grey_guid[i];
  desc->raw.bBitsPerPixel = 8;
  desc->raw.bVariableSize = 0;
}

static void build_uvc_fb_h264_fmt_desc(struct uvc_fb_fmt_desc *desc, UVCL_DescConf *p_conf)
{
  const uint8_t fb_h264_guid[] = {'H', '2', '6', '4', 0x00, 0x00, 0x10, 0x00,
                 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71};
  int i;

  for (i = 0; i < 16; i++)
    desc->raw.guidFormat[i] = fb_h264_guid[i];
  desc->raw.bBitsPerPixel = 0;
  desc->raw.bVariableSize = 1;
}

static void build_uvc_fb_bgr3_fmt_desc(struct uvc_fb_fmt_desc *desc, UVCL_DescConf *p_conf)
{
  const uint8_t fb_rgb888_guid[] = {0x7d, 0xeb, 0x36, 0xe4, 0x4f, 0x52, 0xce, 0x11,
                 0x9f, 0x53, 0x00, 0x20, 0xaf, 0x0b, 0xa7, 0x70};
  int i;

  for (i = 0; i < 16; i++)
    desc->raw.guidFormat[i] = fb_rgb888_guid[i];
  desc->raw.bBitsPerPixel = 24;
  desc->raw.bVariableSize = 0;
}

static void build_uvc_fb_fmt_desc(struct uvc_fb_fmt_desc *desc, struct uvc_desc_head *next,
                                       struct uvc_desc_head *parent, UVCL_DescConf *p_conf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = VS_FORMAT_FRAME_BASED;
  desc->raw.bFormatIndex = 1;
  desc->raw.bNumFrameDescriptors = 1;
  desc->raw.bDefaultFrameIndex = 1;
  desc->raw.bAspectRatioX = 0;
  desc->raw.bAspectRatioY = 0;
  desc->raw.bmInterlaceFlags = 0;
  desc->raw.bCopyProtect = 0;

  /* Fill guidFormat, bBitsPerPixel and bVariableSize according to payload type */
  switch (p_conf->payload_type) {
  case UVCL_PAYLOAD_FB_RGB565:
    build_uvc_fb_rgb565_fmt_desc(desc, p_conf);
    break;
  case UVCL_PAYLOAD_FB_GREY:
    build_uvc_fb_grey_fmt_desc(desc, p_conf);
    break;
  case UVCL_PAYLOAD_FB_H264:
    build_uvc_fb_h264_fmt_desc(desc, p_conf);
    break;
  case UVCL_PAYLOAD_FB_BGR3:
    build_uvc_fb_bgr3_fmt_desc(desc, p_conf);
    break;
  default:
    assert(0);
  }

  append_as_child(parent, &desc->head);
}

static void build_uvc_fb_rgb565_frame_desc(struct uvc_fb_frame_desc *desc, UVCL_DescConf *p_conf)
{
  desc->raw.dwMinBitRate = desc->raw.wWidth * desc->raw.wHeight * p_conf->fps * 16;
  desc->raw.dwMaxBitRate = desc->raw.dwMinBitRate;
  desc->raw.dwBytesPerLine = desc->raw.wWidth * 2;
}

static void build_uvc_fb_grey_frame_desc(struct uvc_fb_frame_desc *desc, UVCL_DescConf *p_conf)
{
  desc->raw.dwMinBitRate = desc->raw.wWidth * desc->raw.wHeight * p_conf->fps * 8;
  desc->raw.dwMaxBitRate = desc->raw.dwMinBitRate;
  desc->raw.dwBytesPerLine = desc->raw.wWidth * 1;
}

static void build_uvc_fb_h264_frame_desc(struct uvc_fb_frame_desc *desc, UVCL_DescConf *p_conf)
{
  desc->raw.dwMinBitRate = desc->raw.wWidth * desc->raw.wHeight * p_conf->fps;
  desc->raw.dwMaxBitRate = desc->raw.dwMinBitRate;
  desc->raw.dwBytesPerLine = 0;
}

static void build_uvc_fb_bgr3_frame_desc(struct uvc_fb_frame_desc *desc, UVCL_DescConf *p_conf)
{
  desc->raw.dwMinBitRate = desc->raw.wWidth * desc->raw.wHeight * p_conf->fps * 24;
  desc->raw.dwMaxBitRate = desc->raw.dwMinBitRate;
  desc->raw.dwBytesPerLine = desc->raw.wWidth * 3;
}

static void build_uvc_fb_frame_desc(struct uvc_fb_frame_desc *desc, struct uvc_desc_head *next,
                                         struct uvc_desc_head *parent, UVCL_DescConf *p_conf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);;
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = VS_FRAME_FRAME_BASED;
  desc->raw.bFrameIndex = 1;
  desc->raw.bmCapabilities = 2;
  desc->raw.wWidth = p_conf->width;
  desc->raw.wHeight = p_conf->height;
  desc->raw.dwDefaultFrameInterval = 10000000 / p_conf->fps;
  desc->raw.bFrameIntervalType = 1;
  desc->raw.dwFrameInterval[0] = 10000000 / p_conf->fps;

  /* Fill dwMinBitRate, dwMaxBitRate and dwBytesPerLine according to payload type */
  switch (p_conf->payload_type) {
  case UVCL_PAYLOAD_FB_RGB565:
    build_uvc_fb_rgb565_frame_desc(desc, p_conf);
    break;
  case UVCL_PAYLOAD_FB_GREY:
    build_uvc_fb_grey_frame_desc(desc, p_conf);
    break;
  case UVCL_PAYLOAD_FB_H264:
    build_uvc_fb_h264_frame_desc(desc, p_conf);
    break;
  case UVCL_PAYLOAD_FB_BGR3:
    build_uvc_fb_bgr3_frame_desc(desc, p_conf);
    break;
  default:
    assert(0);
  }

  append_as_child(parent, &desc->head);
}

static void build_xx_fb_conf_desc(struct uvc_fb_conf_desc *desc, UVCL_DescConf *p_conf, int wMaxPacketSize)
{
  build_uvc_conf_desc(&desc->conf_desc, &desc->iad_desc.head, p_conf);
    build_uvc_iad_desc(&desc->iad_desc, &desc->std_vc_desc.head, &desc->conf_desc.head, p_conf);
      build_uvc_std_vc_desc(&desc->std_vc_desc, &desc->class_vc_desc.head, &desc->iad_desc.head, p_conf);
        build_uvc_class_vc_desc(&desc->class_vc_desc, &desc->cam_desc.head, &desc->std_vc_desc.head, p_conf);
          build_uvc_camera_terminal_desc(&desc->cam_desc, &desc->tt_desc.head, &desc->class_vc_desc.head, p_conf);
          build_uvc_output_term_desc(&desc->tt_desc, &desc->std_vs_alt0_desc.head, &desc->class_vc_desc.head, p_conf);
      build_uvc_std_vs_desc(&desc->std_vs_alt0_desc, &desc->vs_input_desc.head, &desc->iad_desc.head, p_conf, 0, 0);
        build_uvc_vs_input_desc(&desc->vs_input_desc, &desc->fb_fmt_desc.head, &desc->std_vs_alt0_desc.head, p_conf, 1);
          build_uvc_fb_fmt_desc(&desc->fb_fmt_desc, &desc->fb_frame_desc.head, &desc->vs_input_desc.head, p_conf);
            build_uvc_fb_frame_desc(&desc->fb_frame_desc, &desc->color_desc.head, &desc->fb_fmt_desc.head, p_conf);
          build_uvc_color_desc(&desc->color_desc, &desc->std_vs_alt1_desc.head, &desc->fb_fmt_desc.head, p_conf);
      build_uvc_std_vs_desc(&desc->std_vs_alt1_desc, &desc->ep_desc.head, &desc->iad_desc.head, p_conf, 1, 1);
        build_uvc_vs_ep_desc(&desc->ep_desc, NULL, &desc->std_vs_alt1_desc.head, p_conf, wMaxPacketSize);
}

static int UVCL_build_hs_fb_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  struct uvc_fb_conf_desc desc = { 0 };
  int wMaxPacketSize = 1024 | ((USBL_PACKET_PER_MICRO_FRAME - 1) << 11);

  assert(USBL_PACKET_PER_MICRO_FRAME >= 1 && USBL_PACKET_PER_MICRO_FRAME <= 3);

  build_xx_fb_conf_desc(&desc, p_conf, wMaxPacketSize);
  update(&desc.conf_desc.head);
  return generate(&desc.conf_desc.head, p_dst, dst_len);

  assert(0);
}

static int UVCL_build_fs_fb_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  struct uvc_fb_conf_desc desc = { 0 };

  build_xx_fb_conf_desc(&desc, p_conf, 1023);
  update(&desc.conf_desc.head);
  return generate(&desc.conf_desc.head, p_dst, dst_len);}

static int UVCL_build_fb_rgb565_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  if (p_conf->is_hs)
    return UVCL_build_hs_fb_configuration_desc(p_dst, dst_len, p_conf);
  else
    return UVCL_build_fs_fb_configuration_desc(p_dst, dst_len, p_conf);
}

static int UVCL_build_fb_grey_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  if (p_conf->is_hs)
    return UVCL_build_hs_fb_configuration_desc(p_dst, dst_len, p_conf);
  else
    return UVCL_build_fs_fb_configuration_desc(p_dst, dst_len, p_conf);
}

static int UVCL_build_fb_h264_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  if (p_conf->is_hs)
    return UVCL_build_hs_fb_configuration_desc(p_dst, dst_len, p_conf);
  else
    return UVCL_build_fs_fb_configuration_desc(p_dst, dst_len, p_conf);
}

static int UVCL_build_fb_bgr3_configuration_desc(uint8_t *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  if (p_conf->is_hs)
    return UVCL_build_hs_fb_configuration_desc(p_dst, dst_len, p_conf);
  else
    return UVCL_build_fs_fb_configuration_desc(p_dst, dst_len, p_conf);
}

int UVCL_get_device_desc(void *p_dst, int dst_len, int idx_manufacturer, int idx_product, int idx_serial)
{
  struct uvc_dev_desc dev_desc = { 0 };

  build_dev_desc(&dev_desc, NULL, NULL, idx_manufacturer, idx_product, idx_serial);
  update(&dev_desc.head);

  return generate(&dev_desc.head, p_dst, dst_len);
}

int UVCL_get_device_qualifier_desc(void *p_dst, int dst_len)
{
  uint8_t *dst = p_dst;

  if (dst_len < 10)
    return -1;

  dst[0] = 0x0a; /* bLength = 10 */
  dst[1] = 0x06; /* dev qualifier desc type */
  dst[2] = 0x00; /* bcdUSB low */
  dst[3] = 0x02; /* bcdUSB high */
  dst[4] = 0xEF; /* bDeviceClass */
  dst[5] = 0x02; /* bDeviceSubClass */
  dst[6] = 0x01; /* bDeviceProtocol */
  dst[7] = 0x40; /* bMaxPacketSize0 64 */
  dst[8] = 0x01; /* bNumConfigurations */
  dst[9] = 0x00; /* bReserved */

  return dst[0];
}

int UVCL_get_lang_string_desc(void *p_dst, int dst_len)
{
  uint8_t *dst = p_dst;

  if (dst_len < 4)
    return -1;

  dst[0] = 0x04; /* bLength = 4 */
  dst[1] = 0x03; /* string descriptor */
  dst[2] = 0x09; /* United States */
  dst[3] = 0x04; /* United States */

  return dst[0];
}

int UVCL_get_manufacturer_string_desc(void *p_dst, int dst_len)
{
  return UVCL_get_string_desc(p_dst, dst_len, DEV_MANUFACTURER_STRING);
}

int UVCL_get_product_string_desc(void *p_dst, int dst_len)
{
  return UVCL_get_string_desc(p_dst, dst_len, DEV_PRODUCT_STRING);
}

int UVCL_get_serial_string_desc(void *p_dst, int dst_len)
{
  char serial[13];

  gen_serial(serial);
  return UVCL_get_string_desc(p_dst, dst_len, serial);
}

int UVCL_get_configuration_desc(void *p_dst, int dst_len, UVCL_DescConf *p_conf)
{
  switch (p_conf->payload_type) {
  case UVCL_PAYLOAD_UNCOMPRESSED_YUY2:
    return UVCL_build_yuv422_configuration_desc(p_dst, dst_len, p_conf);
    break;
  case UVCL_PAYLOAD_JPEG:
    return UVCL_build_jpeg_configuration_desc(p_dst, dst_len, p_conf);
    break;
  case UVCL_PAYLOAD_FB_RGB565:
    return UVCL_build_fb_rgb565_configuration_desc(p_dst, dst_len, p_conf);
    break;
  case UVCL_PAYLOAD_FB_GREY:
    return UVCL_build_fb_grey_configuration_desc(p_dst, dst_len, p_conf);
    break;
  case UVCL_PAYLOAD_FB_H264:
    return UVCL_build_fb_h264_configuration_desc(p_dst, dst_len, p_conf);
    break;
  case UVCL_PAYLOAD_FB_BGR3:
    return UVCL_build_fb_bgr3_configuration_desc(p_dst, dst_len, p_conf);
    break;
  default:
    return -1;
  }
}
