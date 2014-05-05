/*
 * File      : startup.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://openlab.rt-thread.com/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-08-31     Bernard      first implementation
 */

#include <rthw.h>
#include <rtthread.h>

#include <stm32f2xx.h>
#include "board.h"

/**
 * @addtogroup STM32
 */

/*@{*/

extern int  rt_application_init(void);



#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
	rt_kprintf("\n\r Wrong parameter value detected on\r\n");
	rt_kprintf("       file  %s\r\n", file);
	rt_kprintf("       line  %d\r\n", line);

	while (1) ;
}
#endif
void uart_config(void)
{
USART_InitTypeDef USART_InitStructure;
USART_InitStructure.USART_BaudRate = 115200;

    USART_InitStructure.USART_WordLength = USART_WordLength_8b;

   USART_InitStructure.USART_StopBits = USART_StopBits_1;

    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(USART2, ENABLE);
    /* enable interrupt */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	}
/**
 * This function will startup RT-Thread RTOS.
 */
void rtthread_startup(void)
{
	/* init board */
	rt_hw_board_init();

	/* show version */
	rt_show_version();

	/* init tick */
	rt_system_tick_init();

	/* init kernel object */
	rt_system_object_init();

	/* init timer system */
	rt_system_timer_init();

#ifdef RT_USING_HEAP
    rt_system_heap_init((void*)HEAP_BEGIN, (void*)HEAP_END);
#endif

	/* init scheduler system */
	rt_system_scheduler_init();

#ifdef RT_USING_DFS
	/* init sdcard driver */
	rt_hw_msd_init();
#endif
   	wifi_low_level();
	//rt_hw_rtc_init();
uart_config();
	/* init all device */
	rt_device_init_all();

	/* init application */
	rt_application_init();

    /* init timer thread */
    rt_system_timer_thread_init();

	/* init idle thread */
	rt_thread_idle_init();

	/* start scheduler */
	rt_system_scheduler_start();

	/* never reach here */
	return ;
}

int main(void)
{
	/* disable interrupt first */
	//rt_hw_interrupt_disable();

	/* startup RT-Thread RTOS */
	rtthread_startup();

	return 0;
}

/*@}*/
