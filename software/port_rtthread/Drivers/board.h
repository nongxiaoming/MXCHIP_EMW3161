/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-02-28     Bernard      Add Configurator tag
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <stm32f2xx.h>

/* board configuration */

// <o> Internal SRAM memory size[Kbytes] <8-128>
//	<i>Default: 128
#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN  ((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define HEAP_BEGIN  (__segment_end("HEAP"))
#else
extern int __bss_end;
#define HEAP_BEGIN  ((void *)&__bss_end)
#endif
#define HEAP_END    (void*)(0x20000000 + 0x20000)

//#define RT_USING_UART1
#define RT_USING_UART2
//#define RT_USING_UART3
//#define RT_USING_UART6


void rt_hw_board_init(void);

void wifi_low_level(void);
void rt_hw_usart_init(void);


#ifdef __cplusplus
}
#endif

#endif
