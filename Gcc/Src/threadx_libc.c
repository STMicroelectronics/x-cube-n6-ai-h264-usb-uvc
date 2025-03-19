/**
  ******************************************************************************
  * @file    threadx_libc.c
  * @author  MDG Application Team
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the ST_LICENSE.md file
  * in the root directory of this software component.
  * If no ST_LICENSE.md file comes with this software, it is provided AS-IS.
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

static int is_tx_rt_init_done;
static TX_MUTEX libc_lock;

static void threadx_libc_init()
{
  int ret;

  ret = tx_mutex_create(&libc_lock, NULL, TX_INHERIT);
  assert(ret == 0);

  is_tx_rt_init_done = 1;
}

void __malloc_lock (struct _reent *reent)
{
  int ret;

  if (TX_THREAD_GET_SYSTEM_STATE() != TX_INITIALIZE_IS_FINISHED)
    return ;

  if (!is_tx_rt_init_done)
    threadx_libc_init();

  assert(is_tx_rt_init_done);
  assert(!IS_IRQ_MODE());

  ret = tx_mutex_get(&libc_lock, TX_WAIT_FOREVER);
  assert(ret == 0);
}

void __malloc_unlock (struct _reent *reent)
{
  int ret;

  if (TX_THREAD_GET_SYSTEM_STATE() != TX_INITIALIZE_IS_FINISHED)
    return ;

  if (!is_tx_rt_init_done)
    threadx_libc_init();

  assert(is_tx_rt_init_done);
  assert(!IS_IRQ_MODE());

  ret = tx_mutex_put(&libc_lock);
  assert(ret == 0);
}
