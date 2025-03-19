 /**
 ******************************************************************************
 * @file    threadx_hal.c
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

#include <assert.h>
#include <stdint.h>

#include "cmsis_compiler.h"
#include "tx_api.h"
#include "tx_thread.h"
#include "tx_initialize.h"

#define IS_IRQ_MODE()     (__get_IPSR() != 0U)

uint32_t HAL_GetTick()
{
  return tx_time_get();
}

void HAL_Delay(uint32_t Delay)
{
  if (IS_IRQ_MODE())
    assert(0);

  tx_thread_sleep(Delay);
}

int HAL_InitTick(uint32_t TickPriority)
{
  return 0;
}
