/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 */

#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "mxchipWNET.h"
/**
 * @addtogroup STM32
 */

/*@{*/

/*******************************************************************************
* Function Name  :  wifi_low_level
* Description    : wifi low level init.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void wifi_low_level(void)
{
	lib_config_t config;
	config.tcp_buf_dynamic = 1;
  config.tcp_max_connection_num = 12;
  config.tcp_rx_size = 2048;
  config.tcp_tx_size = 2048;
  config.hw_watchdog = 0;
  config.wifi_channel = WIFI_CHANEEL_1_13;
	lib_config(&config);
	mxchipInit();
}

/*******************************************************************************
 * Function Name  : SysTick_Configuration
 * Description    : Configures the SysTick for OS tick.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void  SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	rt_uint32_t         cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (rt_uint32_t)rcc_clocks.HCLK_Frequency / RT_TICK_PER_SECOND;

	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
}

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
		systick_irq();    
	  NoOS_systick_irq();
	/* enter interrupt */
	rt_interrupt_enter();
	
	rt_tick_increase();
	
	/* leave interrupt */
	rt_interrupt_leave();
}

/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
	/* Configure the SysTick */
	SysTick_Configuration();

	rt_hw_usart_init();
#ifdef RT_USING_CONSOLE
	rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
	 
}

/*@}*/
