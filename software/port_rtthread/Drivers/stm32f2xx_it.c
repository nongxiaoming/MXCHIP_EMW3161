/**
  ******************************************************************************
  * @file    stm32f2xx_it.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    06-June-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_it.h"
#include "mxchipWNET.h"
#include <rtthread.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/


/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

__asm void asm_dump(void)
{
	 IMPORT hard_fault_handler_c
	 TST LR, #4
	 ITE EQ
	 MRSEQ R0, MSP
	 MRSNE R0, PSP
	 B hard_fault_handler_c
 }

///**
//  * @brief  This function handles Hard Fault exception.
//  * @param  None
//  * @retval None
//  */
//void HardFault_Handler(void)
//{
//	asm_dump();
//}


/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

///**
//  * @brief  This function handles PendSVC exception.
//  * @param  None
//  * @retval None
//  */
//void PendSV_Handler(void)
//{
//}

///**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @retval None
//  */
//void SysTick_Handler(void)
//{
//	systick_irq();    
//	NoOS_systick_irq();
//}

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles EXTI15_10.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	//rt_interrupt_enter();
  gpio_irq();
	//rt_interrupt_leave();
}
void EXTI0_IRQHandler(void)
{
	//rt_interrupt_enter();
	gpio_irq(); //SDIO OOB interrupt
	//rt_interrupt_leave();
}

void EXTI3_IRQHandler(void)
{
	//rt_interrupt_enter();
	gpio_irq(); //User defined external interrupt, EMW3162 button 1: PA3
	//rt_interrupt_leave();
}
void EXTI9_5_IRQHandler(void)
{
	//rt_interrupt_enter();
  gpio_irq();//User defined external interrupt, EMW3161 button 1: PH9
	//rt_interrupt_leave();
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/
 /*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{
	//rt_interrupt_enter();
  /* Process All SDIO Interrupt Sources */
  sdio_irq();
	//rt_interrupt_leave();
}

void DMA2_Stream3_IRQHandler(void)
{
  // rt_interrupt_enter(); 
	dma_irq();
	//rt_interrupt_leave();
}

/*******************************************************************************
* Function Name  : DMA2_Stream2_IRQHandler
* Description    : This function handles DMA2 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Stream2_IRQHandler(void)
{
  //rt_interrupt_enter();  
	uart_dma_irq();
	//rt_interrupt_leave();
}

void USART6_IRQHandler(void)
{
   // rt_interrupt_enter();
    bt_uart_irq();
	//rt_interrupt_leave();
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
