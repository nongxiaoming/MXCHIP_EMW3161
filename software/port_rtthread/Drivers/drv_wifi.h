/*
 * File      : drv_wifi.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2014, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-03-29     xiaonong      the first version
 */

#ifndef __DRV_WIFI_H
#define __DRV_WIFI_H
#include "mxchipWNET.h"

void stationModeStart(void);
void softAPModeStart(void);
int wifi_thread_init(void);
#endif
