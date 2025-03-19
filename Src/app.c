 /**
 ******************************************************************************
 * @file    app.c
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

#include "app.h"

#include <stdint.h>

#include "app_cam.h"
#include "app_config.h"
#include "app_postprocess.h"
#include "isp_api.h"
#include "ll_aton_runtime.h"
#include "cmw_camera.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_ll_venc.h"
#include "stm32n6570_discovery.h"
#include "tx_api.h"
#include "app_enc.h"
#include "utils.h"
#include "uvcl.h"
#include "draw.h"

#include "figs.h"

#define CACHE_OP(__op__) do { \
  if (is_cache_enable()) { \
    __op__; \
  } \
} while (0)

#define ALIGN_VALUE(_v_,_a_) (((_v_) + (_a_) - 1) & ~((_a_) - 1))

#define DBG_INFO_FONT font_12
#define CONF_LEVEL_FONT font_16
#define INF_INFO_FONT font_16
#define OBJ_RECT_COLOR 0xffffffff

#define BQUEUE_MAX_BUFFERS 2
#define CPU_LOAD_HISTORY_DEPTH 8

#define CAPTURE_BUFFER_NB (CAPTURE_DELAY + 2)

/* venc conf */
#define VENC_MAX_WIDTH 1280
#define VENC_MAX_HEIGHT 720
#define VENC_OUT_BUFFER_SIZE (255 * 1024)

/* Model Related Info */
#define NN_BUFFER_OUT_SIZE 5880

/* Align so we are sure nn_output_buffers[0] and nn_output_buffers[1] are aligned on 32 bytes */
#define NN_BUFFER_OUT_SIZE_ALIGN ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32)

typedef struct {
  int last;
  int total;
  uint64_t acc;
  float mean;
} time_stat_t;

typedef struct {
  time_stat_t nn_total_time;
  time_stat_t nn_inference_time;
  time_stat_t disp_total_time;
  time_stat_t nn_pp_time;
  time_stat_t disp_display_time;
  time_stat_t disp_enc_time;
} stat_info_t;

typedef struct {
  float conf;
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
} box_t;

typedef struct {
  TX_SEMAPHORE free;
  TX_SEMAPHORE ready;
  int buffer_nb;
  uint8_t *buffers[BQUEUE_MAX_BUFFERS];
  int free_idx;
  int ready_idx;
} bqueue_t;

typedef struct {
  uint64_t current_total;
  uint64_t current_thread_total;
  uint64_t prev_total;
  uint64_t prev_thread_total;
  struct {
    uint64_t total;
    uint64_t thread;
    uint32_t tick;
  } history[CPU_LOAD_HISTORY_DEPTH];
} cpuload_info_t;

/* Globals */
/* display */
static DRAW_Font_t font_12;
static DRAW_Font_t font_16;
static TX_MUTEX stat_info_lock;
static stat_info_t stat_info;
static cpuload_info_t cpu_load;

/* dma2d */
static TX_MUTEX dma2d_lock;
static TX_SEMAPHORE dma2d_sem;
/* Store current DMA2D_HandleTypeDef instance so we can propagate to irq handler */
static DMA2D_HandleTypeDef *dma2d_current;

/* capture buffers */
static uint8_t capture_buffer[CAPTURE_BUFFER_NB][VENC_MAX_WIDTH * VENC_MAX_HEIGHT * CAPTURE_BPP] ALIGN_32 IN_PSRAM;
static int capture_buffer_disp_idx = 1;
static int capture_buffer_capt_idx = 0;

/* model */
LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(Default);
 /* nn input buffers */
static uint8_t nn_input_buffers[2][NN_WIDTH * NN_HEIGHT * NN_BPP] ALIGN_32 IN_PSRAM;
static bqueue_t nn_input_queue;
 /* nn output buffers */
static uint8_t nn_output_buffers[2][NN_BUFFER_OUT_SIZE_ALIGN] ALIGN_32;
static bqueue_t nn_output_queue;

/* venc */
static uint8_t venc_out_buffer[VENC_OUT_BUFFER_SIZE] ALIGN_32 UNCACHED;
static uint8_t uvc_in_buffers[VENC_OUT_BUFFER_SIZE] ALIGN_32;

/* uvc */
static struct uvcl_callbacks uvcl_cbs;
static int uvc_is_active;
static volatile int buffer_flying;
static int force_intra;

 /* threads */
  /* nn thread */
static TX_THREAD nn_thread;
static uint8_t nn_tread_stack[4096];
  /* display thread */
static TX_THREAD dp_thread;
static uint8_t dp_tread_stack[4096];
  /* isp thread */
static TX_THREAD isp_thread;
static uint8_t isp_tread_stack[4096];
static TX_SEMAPHORE isp_sem;

static int is_cache_enable()
{
#if defined(USE_DCACHE)
  return 1;
#else
  return 0;
#endif
}

static void cpuload_init(cpuload_info_t *cpu_load)
{
  memset(cpu_load, 0, sizeof(cpuload_info_t));
}

static void cpuload_update(cpuload_info_t *cpu_load)
{
  EXECUTION_TIME thread_total;
  EXECUTION_TIME isr;
  EXECUTION_TIME idle;
  int i;

  cpu_load->history[1] = cpu_load->history[0];

  _tx_execution_thread_total_time_get(&thread_total);
  _tx_execution_isr_time_get(&isr);
  _tx_execution_idle_time_get(&idle);

  cpu_load->history[0].total = thread_total + isr + idle;
  cpu_load->history[0].thread = thread_total;
  cpu_load->history[0].tick = HAL_GetTick();

  if (cpu_load->history[1].tick - cpu_load->history[2].tick < 1000)
    return ;

  for (i = 0; i < CPU_LOAD_HISTORY_DEPTH - 2; i++)
    cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i] = cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i - 1];
}

static void cpuload_get_info(cpuload_info_t *cpu_load, float *cpu_load_last, float *cpu_load_last_second,
                             float *cpu_load_last_five_seconds)
{
  if (cpu_load_last)
    *cpu_load_last = 100.0 * (cpu_load->history[0].thread - cpu_load->history[1].thread) /
                     (cpu_load->history[0].total - cpu_load->history[1].total);
  if (cpu_load_last_second)
    *cpu_load_last_second = 100.0 * (cpu_load->history[2].thread - cpu_load->history[3].thread) /
                     (cpu_load->history[2].total - cpu_load->history[3].total);
  if (cpu_load_last_five_seconds)
    *cpu_load_last_five_seconds = 100.0 * (cpu_load->history[2].thread - cpu_load->history[7].thread) /
                     (cpu_load->history[2].total - cpu_load->history[7].total);
}

static void time_stat_update(time_stat_t *p_stat, int value)
{
  tx_mutex_get(&stat_info_lock, TX_WAIT_FOREVER);
  p_stat->last = value;
  p_stat->acc += value;
  p_stat->total++;
  p_stat->mean = (float)p_stat->acc / p_stat->total;
  tx_mutex_put(&stat_info_lock);
}

static void stat_info_copy(stat_info_t *copy)
{
  tx_mutex_get(&stat_info_lock, TX_WAIT_FOREVER);
  *copy = stat_info;
  tx_mutex_put(&stat_info_lock);
}

static int bqueue_init(bqueue_t *bq, int buffer_nb, uint8_t **buffers)
{
  int ret;
  int i;

  if (buffer_nb > BQUEUE_MAX_BUFFERS)
    return -1;

  ret = tx_semaphore_create(&bq->free, NULL, buffer_nb);
  if (ret)
    goto free_sem_error;
  ret = tx_semaphore_create(&bq->ready, NULL, 0);
  if (ret)
    goto ready_sem_error;

  bq->buffer_nb = buffer_nb;
  for (i = 0; i < buffer_nb; i++) {
    assert(buffers[i]);
    bq->buffers[i] = buffers[i];
  }
  bq->free_idx = 0;
  bq->ready_idx = 0;

  return 0;

ready_sem_error:
  tx_semaphore_delete(&bq->free);
free_sem_error:
  return -1;
}

static uint8_t *bqueue_get_free(bqueue_t *bq, int is_blocking)
{
  uint8_t *res;
  int ret;

  ret = tx_semaphore_get(&bq->free, is_blocking ? TX_WAIT_FOREVER : TX_NO_WAIT);
  if (ret == TX_NO_INSTANCE)
    return NULL;
  assert(ret == 0);

  res = bq->buffers[bq->free_idx];
  bq->free_idx = (bq->free_idx + 1) % bq->buffer_nb;

  return res;
}

static void bqueue_put_free(bqueue_t *bq)
{
  int ret;

  ret = tx_semaphore_put(&bq->free);
  assert(ret == 0);
}

static uint8_t *bqueue_get_ready(bqueue_t *bq)
{
  uint8_t *res;
  int ret;

  ret = tx_semaphore_get(&bq->ready, TX_WAIT_FOREVER);
  assert(ret == 0);

  res = bq->buffers[bq->ready_idx];
  bq->ready_idx = (bq->ready_idx + 1) % bq->buffer_nb;

  return res;
}

static void bqueue_put_ready(bqueue_t *bq)
{
  int ret;

  ret = tx_semaphore_put(&bq->ready);
  assert(ret == 0);
}

static void app_main_pipe_frame_event()
{
  int next_disp_idx = (capture_buffer_disp_idx + 1) % CAPTURE_BUFFER_NB;
  int next_capt_idx = (capture_buffer_capt_idx + 1) % CAPTURE_BUFFER_NB;
  int ret;

  ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE1,
                                         DCMIPP_MEMORY_ADDRESS_0, (uint32_t) capture_buffer[next_capt_idx]);
  assert(ret == HAL_OK);

  capture_buffer_disp_idx = next_disp_idx;
  capture_buffer_capt_idx = next_capt_idx;
}

static void app_ancillary_pipe_frame_event()
{
  uint8_t *next_buffer;
  int ret;

  next_buffer = bqueue_get_free(&nn_input_queue, 0);
  if (next_buffer) {
    ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE2,
                                           DCMIPP_MEMORY_ADDRESS_0, (uint32_t) next_buffer);
    assert(ret == HAL_OK);
    bqueue_put_ready(&nn_input_queue);
  }
}

static void app_main_pipe_vsync_event()
{
  int ret;

  ret = tx_semaphore_put(&isp_sem);
  assert(ret == 0);
}

static void nn_thread_fct(ULONG arg)
{
  const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info_Default();
  const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info_Default();
  uint32_t nn_period_ms;
  uint32_t nn_period[2];
  uint8_t *nn_pipe_dst;
  uint32_t nn_out_len;
  uint32_t nn_in_len;
  uint32_t total_ts;
  uint32_t ts;
  int ret;

  (void) nn_period_ms;

  /* setup inout buffers and size */
  nn_in_len = LL_Buffer_len(&nn_in_info[0]);
  nn_out_len = LL_Buffer_len(&nn_out_info[0]);
  //printf("nn_out_len = %d\n", nn_out_len);
  assert(nn_out_len == NN_BUFFER_OUT_SIZE);

  /*** App Loop ***************************************************************/
  nn_period[1] = HAL_GetTick();

  nn_pipe_dst = bqueue_get_free(&nn_input_queue, 0);
  assert(nn_pipe_dst);
  CAM_NNPipe_Start(nn_pipe_dst, CMW_MODE_CONTINUOUS);
  while (1)
  {
    uint8_t *capture_buffer;
    uint8_t *output_buffer;

    nn_period[0] = nn_period[1];
    nn_period[1] = HAL_GetTick();
    nn_period_ms = nn_period[1] - nn_period[0];

    capture_buffer = bqueue_get_ready(&nn_input_queue);
    assert(capture_buffer);
    output_buffer = bqueue_get_free(&nn_output_queue, 1);
    assert(output_buffer);

    total_ts = HAL_GetTick();
    /* run ATON inference */
    ts = HAL_GetTick();
     /* Note that we don't need to clean/invalidate those input buffers since they are only access in hardware */
    ret = LL_ATON_Set_User_Input_Buffer_Default(0, capture_buffer, nn_in_len);
     /* Invalidate output buffer before Hw access it */
    CACHE_OP(SCB_InvalidateDCache_by_Addr(output_buffer, nn_out_len));
    ret = LL_ATON_Set_User_Output_Buffer_Default(0, output_buffer, nn_out_len);
    assert(ret == LL_ATON_User_IO_NOERROR);
    LL_ATON_RT_Main(&NN_Instance_Default);
    time_stat_update(&stat_info.nn_inference_time, HAL_GetTick() - ts);

    /* release buffers */
    bqueue_put_free(&nn_input_queue);
    bqueue_put_ready(&nn_output_queue);

    time_stat_update(&stat_info.nn_total_time, HAL_GetTick() - total_ts);
  }
}

static size_t encode_display(int is_intra_force, uint8_t *p_buffer)
{
  size_t res;

  res = ENC_EncodeFrame(p_buffer, venc_out_buffer, VENC_OUT_BUFFER_SIZE, is_intra_force);
  /* usb is lagging, drop frame and force next to be an Intra */
  if ((int)res > 0 && buffer_flying) {
    force_intra = 1;
    return -1;
  }

  /* encoder failed certainly due to output buffer too small */
  if ((int)res <= 0)
    return res;

  memcpy(&uvc_in_buffers, venc_out_buffer, res);

  return res;
}

static int send_display(int len)
{
  int ret;

  buffer_flying = 1;
  ret = UVCL_ShowFrame(uvc_in_buffers, len);
  if (ret != 0)
    buffer_flying = 0;

  return ret;
}

static int build_display_inference_info(uint8_t *p_buffer, uint32_t inf_time, int line_nb)
{
  const int offset_x = 16;

  DRAW_PrintfArgbHw(&INF_INFO_FONT, p_buffer, VENC_WIDTH, VENC_HEIGHT, offset_x, line_nb * INF_INFO_FONT.height,
                    " Inference : %4.1f ms ", (double)inf_time);

  return line_nb + 1;
}

static int build_display_cpu_load(uint8_t *p_buffer, int line_nb)
{
  const int offset_x = 16;
  float cpu_load_one_second;

  cpuload_get_info(&cpu_load, NULL, &cpu_load_one_second, NULL);
  DRAW_PrintfArgbHw(&INF_INFO_FONT, p_buffer, VENC_WIDTH, VENC_HEIGHT, offset_x, line_nb * INF_INFO_FONT.height,
                    " Cpu load  : %4.1f  %% ", cpu_load_one_second);
  line_nb++;

  return line_nb;
}

static int clamp_point(int *x, int *y)
{
  int xi = *x;
  int yi = *y;

  if (*x < 0)
    *x = 0;
  if (*y < 0)
    *y = 0;
  if (*x >= VENC_WIDTH)
    *x = VENC_WIDTH - 1;
  if (*y >= VENC_HEIGHT)
    *y = VENC_HEIGHT - 1;

  return (xi != *x) || (yi != *y);
}

static void convert_length(float32_t wi, float32_t hi, int *wo, int *ho)
{
  *wo = (int) (VENC_WIDTH * wi);
  *ho = (int) (VENC_HEIGHT * hi);
}

static void convert_point(float32_t xi, float32_t yi, int *xo, int *yo)
{
  *xo = (int) (VENC_WIDTH * xi);
  *yo = (int) (VENC_HEIGHT * yi);
}

static void cvt_nn_box_to_dp_box(od_pp_outBuffer_t *detect, box_t *box_dp)
{
  int xc, yc;
  int x0, y0;
  int x1, y1;
  int w, h;

  convert_point(detect->x_center, detect->y_center, &xc, &yc);
  convert_length(detect->width, detect->height, &w, &h);
  x0 = xc - (w + 1) / 2;
  y0 = yc - (h + 1) / 2;
  x1 = xc + (w + 1) / 2;
  y1 = yc + (h + 1) / 2;
  clamp_point(&x0, &y0);
  clamp_point(&x1, &y1);

  box_dp->x = x0;
  box_dp->y = y0;
  box_dp->w = x1 - x0;
  box_dp->h = y1 - y0;
  box_dp->conf = detect->conf;
}

static void draw_box(uint8_t *p_buffer, od_pp_outBuffer_t *box_nn)
{
  box_t box_disp;

  cvt_nn_box_to_dp_box(box_nn, &box_disp);
  DRAW_RectArgbHw(p_buffer, VENC_WIDTH, VENC_HEIGHT, box_disp.x, box_disp.y, box_disp.w, box_disp.h, OBJ_RECT_COLOR);
  DRAW_PrintfArgbHw(&CONF_LEVEL_FONT, p_buffer, VENC_WIDTH, VENC_HEIGHT, box_disp.x, box_disp.y, "%5.1f %%",
                    box_disp.conf * 100);
}

static void time_stat_display(time_stat_t *p_stat, uint8_t *p_buffer, char *label, int line_nb, int indent)
{
  int offset = VENC_WIDTH - 41 * DBG_INFO_FONT.width;

  DRAW_PrintfArgbHw(&DBG_INFO_FONT, p_buffer, VENC_WIDTH, VENC_HEIGHT, offset, line_nb * DBG_INFO_FONT.height,
                    "%*s%s : %3d ms / %5.1f ms ", indent + 1, "", label, p_stat->last, p_stat->mean);
}

static int build_display_nn_dbg(uint8_t *p_buffer, stat_info_t *si, int line_nb)
{
  time_stat_display(&si->nn_total_time, p_buffer,     "NN thread stats  ", line_nb++, 0);
  time_stat_display(&si->nn_inference_time, p_buffer, "inference    ", line_nb++, 4);

  return line_nb;
}

static int build_display_disp_dbg(uint8_t *p_buffer, stat_info_t *si, int line_nb)
{
  time_stat_display(&si->disp_total_time, p_buffer,   "DISP thread stats", line_nb++, 0);
  time_stat_display(&si->nn_pp_time, p_buffer,        "pp           " , line_nb++, 4);
  time_stat_display(&si->disp_display_time, p_buffer, "display      ", line_nb++, 4);
  time_stat_display(&si->disp_enc_time, p_buffer,     "encode       ", line_nb++, 4);

  return line_nb;
}

static int update_and_capture_debug_enabled()
{
  static int prev_button_state = GPIO_PIN_RESET;
  static int display_debug_enabled = 0;
  int cur_button_state;

  cur_button_state = BSP_PB_GetState(BUTTON_USER1);
  if (cur_button_state == GPIO_PIN_SET && prev_button_state == GPIO_PIN_RESET)
    display_debug_enabled = !display_debug_enabled;
  prev_button_state = cur_button_state;

  return display_debug_enabled;
}

static void build_display_stat_info(uint8_t *p_buffer, stat_info_t *si)
{
  int line_nb = 1;

  if (!update_and_capture_debug_enabled())
    return ;

  line_nb = build_display_nn_dbg(p_buffer, si, line_nb);
  line_nb = build_display_disp_dbg(p_buffer, si, line_nb);
}

static void build_display(uint8_t *p_buffer, od_pp_out_t *pp_out)
{
  const uint8_t *fig_array[] = {fig0, fig1, fig2, fig3, fig4, fig5, fig6, fig7, fig8, fig9};
  int line_nb = VENC_HEIGHT / INF_INFO_FONT.height - 4;
  stat_info_t si_copy;
  int nb;
  int i;

  stat_info_copy(&si_copy);

  for (i = 0; i < pp_out->nb_detect; i++)
    draw_box(p_buffer, &pp_out->pOutBuff[i]);

  line_nb = build_display_inference_info(p_buffer, si_copy.nn_inference_time.last, line_nb);
  line_nb = build_display_cpu_load(p_buffer, line_nb);

  nb = MIN(pp_out->nb_detect, ARRAY_NB(fig_array) - 1);
  DRAW_CopyArgbHW(p_buffer, VENC_WIDTH, VENC_HEIGHT, (uint8_t *) fig_array[nb], 64, 64, 16, 16);

  build_display_stat_info(p_buffer, &si_copy);
}

static void display_frame(int is_intra_force, od_pp_out_t *pp_out)
{
  uint8_t *dp_buffer = capture_buffer[capture_buffer_disp_idx];
  uint32_t ts;
  int len;

  ts = HAL_GetTick();
  build_display(dp_buffer, pp_out);
  time_stat_update(&stat_info.disp_display_time, HAL_GetTick() - ts);

  ts = HAL_GetTick();
  len = encode_display(is_intra_force, dp_buffer);
  time_stat_update(&stat_info.disp_enc_time, HAL_GetTick() - ts);

  if (len > 0)
    send_display(len);
}

static int display_new_frame(od_pp_out_t *pp_out)
{
  static int uvc_is_active_prev = 0;
  int uvc_is_active_local = uvc_is_active;

  if (uvc_is_active_local) {
    display_frame(!uvc_is_active_prev || force_intra, pp_out);
    force_intra = 0;
  }

  uvc_is_active_prev = uvc_is_active_local;

  return uvc_is_active_local;
}

static void dp_thread_fct(ULONG arg)
{
#if POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V2_UF
  yolov2_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V5_UU
  yolov5_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V8_UF || POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V8_UI
  yolov8_pp_static_param_t pp_params;
#else
    #error "PostProcessing type not supported"
#endif
  od_pp_out_t pp_output;
  uint32_t total_ts;
  void *pp_input;
  int is_dp_done;
  uint32_t ts;
  int ret;

  /* setup post process */
  app_postprocess_init(&pp_params);
  while (1)
  {
    uint8_t *output_buffer;

    output_buffer = bqueue_get_ready(&nn_output_queue);
    assert(output_buffer);
    total_ts = HAL_GetTick();

    /* Do post process */
    ts = HAL_GetTick();
    pp_input = (void *) output_buffer;
    pp_output.pOutBuff = NULL;
    ret = app_postprocess_run((void * []){pp_input}, 1, &pp_output, &pp_params);
    assert(ret == AI_OD_POSTPROCESS_ERROR_NO);
    time_stat_update(&stat_info.nn_pp_time, HAL_GetTick() - ts);
    cpuload_update(&cpu_load);

    /* compose + encode + send */
    is_dp_done = display_new_frame(&pp_output);

    if (is_dp_done)
      time_stat_update(&stat_info.disp_total_time, HAL_GetTick() - total_ts);

    bqueue_put_free(&nn_output_queue);
  }
}

static void isp_thread_fct(ULONG arg)
{
  int ret;

  while (1) {
    ret = tx_semaphore_get(&isp_sem, TX_WAIT_FOREVER);
    assert(ret == 0);

    CAM_IspUpdate();
  }
}

static void app_uvc_streaming_active(struct uvcl_callbacks *cbs)
{
  uvc_is_active = 1;
  BSP_LED_On(LED_RED);
}

static void app_uvc_streaming_inactive(struct uvcl_callbacks *cbs)
{
  uvc_is_active = 0;
  BSP_LED_Off(LED_RED);
}

static void app_uvc_frame_release(struct uvcl_callbacks *cbs, void *frame)
{
  assert(buffer_flying);

  buffer_flying = 0;
}

void app_run()
{
  const UINT isp_priority = TX_MAX_PRIORITIES / 2 - 2;
  const UINT dp_priority = TX_MAX_PRIORITIES / 2 + 2;
  const UINT nn_priority = TX_MAX_PRIORITIES / 2 - 1;
  const ULONG time_slice = 10;
  UVCL_Conf_t uvcl_conf = { 0 };
  ENC_Conf_t enc_conf = { 0 };
  int ret;

  printf("Init application\n");
  /* Enable DWT so DWT_CYCCNT works when debugger not attached */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  ret = BSP_PB_Init(BUTTON_USER1, BUTTON_MODE_GPIO);
  assert(ret == BSP_ERROR_NONE);

  cpuload_init(&cpu_load);

  /* create buffer queues */
  ret = bqueue_init(&nn_input_queue, 2, (uint8_t *[2]){nn_input_buffers[0], nn_input_buffers[1]});
  assert(ret == 0);
  ret = bqueue_init(&nn_output_queue, 2, (uint8_t *[2]){nn_output_buffers[0], nn_output_buffers[1]});
  assert(ret == 0);

  /* setup fonts */
  ret = DRAW_FontSetup(&Font12, &font_12);
  assert(ret == 0);
  ret = DRAW_FontSetup(&Font16, &font_16);
  assert(ret == 0);

  /* Enable venc */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
   LL_VENC_Init();

  /*** Camera Init ************************************************************/  
  CAM_Init();

  /* Encoder init */
  enc_conf.width = VENC_WIDTH;
  enc_conf.height = VENC_HEIGHT;
  enc_conf.fps = CAMERA_FPS;
  ENC_Init(&enc_conf);

  /* Uvc init */
  uvcl_conf.width = VENC_WIDTH;
  uvcl_conf.height = VENC_HEIGHT;
  uvcl_conf.fps = CAMERA_FPS;
  uvcl_conf.payload_type = UVCL_PAYLOAD_FB_H264;
  uvcl_conf.is_immediate_mode = 1;
  uvcl_cbs.streaming_active = app_uvc_streaming_active;
  uvcl_cbs.streaming_inactive = app_uvc_streaming_inactive;
  uvcl_cbs.frame_release = app_uvc_frame_release;
  ret = UVCL_Init(USB1_OTG_HS, &uvcl_conf, &uvcl_cbs);

  /* sems + mutex init */
  ret = tx_semaphore_create(&isp_sem, NULL, 0);
  assert(ret == 0);
  ret = tx_semaphore_create(&dma2d_sem, NULL, 0);
  assert(ret == 0);
  ret = tx_mutex_create(&dma2d_lock, NULL, TX_INHERIT);
  assert(ret == TX_SUCCESS);
  ret = tx_mutex_create(&stat_info_lock, NULL, TX_INHERIT);
  assert(ret == TX_SUCCESS);

  /* Start LCD Display camera pipe stream */
  CAM_DisplayPipe_Start(capture_buffer[0], CMW_MODE_CONTINUOUS);

  /* threads init */
  ret = tx_thread_create(&nn_thread, "nn", nn_thread_fct, 0, nn_tread_stack,
                         sizeof(nn_tread_stack), nn_priority, nn_priority, time_slice, TX_AUTO_START);
  assert(ret == TX_SUCCESS);
  ret = tx_thread_create(&dp_thread, "dp", dp_thread_fct, 0, dp_tread_stack,
                         sizeof(dp_tread_stack), dp_priority, dp_priority, time_slice, TX_AUTO_START);
  assert(ret == TX_SUCCESS);
  ret = tx_thread_create(&isp_thread, "isp", isp_thread_fct, 0, isp_tread_stack,
                         sizeof(isp_tread_stack), isp_priority, isp_priority, time_slice, TX_AUTO_START);
  assert(ret == TX_SUCCESS);

  BSP_LED_On(LED_GREEN);
}

int CMW_CAMERA_PIPE_FrameEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    app_main_pipe_frame_event();
  else if (pipe == DCMIPP_PIPE2)
    app_ancillary_pipe_frame_event();

  return HAL_OK;
}

int CMW_CAMERA_PIPE_VsyncEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    app_main_pipe_vsync_event();

  return HAL_OK;
}

void DRAW_HwLock(void *dma2d_handle)
{
  tx_mutex_get(&dma2d_lock, TX_WAIT_FOREVER);
  dma2d_current = dma2d_handle;
}

void DRAW_HwUnlock()
{
  tx_mutex_put(&dma2d_lock);
}

void DRAW_Wfe()
{
  int ret;

  ret = tx_semaphore_get(&dma2d_sem, TX_WAIT_FOREVER);
  assert(ret == 0);
}

void DRAW_Signal()
{
  int ret;

  ret = tx_semaphore_put(&dma2d_sem);
  assert(ret == 0);
}

void DMA2D_IRQHandler(void)
{
  HAL_DMA2D_IRQHandler(dma2d_current);
}
