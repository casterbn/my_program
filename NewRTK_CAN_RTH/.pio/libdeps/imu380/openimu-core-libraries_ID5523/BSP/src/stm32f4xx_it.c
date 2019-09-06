/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "Indices.h"
// #include "sensorsAPI.h"
#include "boardDefinition.h"
#include "osapi.h"
#include "RingBuffer.h"
#include "osresources.h"
#include "rtcm.h"
extern uint8_t TxBuffer[];
extern __IO uint32_t TimeOut;
__IO uint8_t Counter = 0;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

// extern uint8_t GpsRxBuff[GPS_BUFF_SIZE * 3];
// extern RingBuffer GpsRing;

// extern uint8_t CloudRxBuff[GPS_BUFF_SIZE * 3];
// extern RingBuffer CloudRing;

extern uint8_t gpsUartBuf[GPS_BUFF_SIZE];
extern uint8_t CloudUartBuf[GPS_BUFF_SIZE];
extern uint8_t IMUUartBuf[IMU_BUFF_SIZE];
extern uint16_t GpsRxLen;
FIFO_Type Uart3RxFifo;
FIFO_Type Uart5RxFifo;
FIFO_Type Uart2RxFifo;
FIFO_Type Uart1RxFifo;

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

volatile struct sHardFaultStacked
{
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t psr;
} HardFault;

#if defined(__ICCARM__)
// Copies of register values pushed onto stack before invoking HardFault handler

// Copy HardFault register values from stack to debugger-friendly memory locations
static void save_fault_cpu_state(uint32_t *stackPtr)
{
  HardFault.r0 = stackPtr[0];
  HardFault.r1 = stackPtr[1];
  HardFault.r2 = stackPtr[2];
  HardFault.r3 = stackPtr[3];
  HardFault.r12 = stackPtr[4];
  HardFault.lr = stackPtr[5];
  HardFault.pc = stackPtr[6];
  HardFault.psr = stackPtr[7];
}

#define IAR_FUNC_ENTRY_PUSHES 2

#endif // __ICCARM__

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
  /* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
  //volatile uint32_t r0;
  //volatile uint32_t r1;
  //volatile uint32_t r2;
  //volatile uint32_t r3;
  //volatile uint32_t r12;
  //volatile uint32_t lr; /* Link register. */
  //volatile uint32_t pc; /* Program counter. */
  //volatile uint32_t psr;/* Program status register. */

  HardFault.r0 = pulFaultStackAddress[0];
  HardFault.r1 = pulFaultStackAddress[1];
  HardFault.r2 = pulFaultStackAddress[2];
  HardFault.r3 = pulFaultStackAddress[3];

  HardFault.r12 = pulFaultStackAddress[4];
  HardFault.lr = pulFaultStackAddress[5];
  HardFault.pc = pulFaultStackAddress[6];
  HardFault.psr = pulFaultStackAddress[7];

  /* When the following line is hit, the variables contain the register values. */
  for (;;)
    ;
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

#if defined(__ICCARM__)
  // NOTE: this only works with IAR
  if (__get_CONTROL() & 2)
  {
    // Process SP in use.
    save_fault_cpu_state((uint32_t *)__get_PSP() + IAR_FUNC_ENTRY_PUSHES);
  }
  else
  {
    // Main SP in use.
    save_fault_cpu_state((uint32_t *)__get_MSP() + IAR_FUNC_ENTRY_PUSHES);
  }
#else
  __asm volatile(
      " tst lr, #4                                                \n"
      " ite eq                                                    \n"
      " mrseq r0, msp                                             \n"
      " mrsne r0, psp                                             \n"
      " ldr r1, [r0, #24]                                         \n"
      " ldr r2, handler2_address_const                            \n"
      " bx r2                                                     \n"
      " handler2_address_const: .word prvGetRegistersFromStack    \n");
#endif // __ICCARM__

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
static const char *bt_trans_test0 = "bluetooth_test_2019_06_18\n\n";
static const char *bt_trans_test1 = "bluetooth_test_by_aceinna\n\n";
void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */
    // static int xx = 0;
    osSemaphoreRelease(dataAcqSem);
    set_ms(5);
    /* USER CODE END TIM2_IRQn 0 */
    HAL_TIM_IRQHandler(&htim2);
    /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}
/**
* @brief This function handles DMA1 stream0 global interrupt.
*/
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream1 global interrupt.
*/
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream3 global interrupt.
*/
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
  //   if(HAL_DMA_GetState(hdma_usart3_rx))
  //   {
  //     if(using_buf0 ==0)
  //     {
  // 　　     DMA1_Stream3->CMAR = (u32)RxBuf0;
  //          using_buf0 = 1;
  //     }
  //     else
  //     {
  //          DMA1_Stream3->CMAR = (u32)RxBuf1;
  //          using_buf0 = 0;
  //     }
  //     recv_flag = 1;
  //   }

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE END DMA1_Stream5_IRQn 0 */
#if 0
  if(__HAL_DMA_GET_FLAG(&hdma_usart2_tx,DMA_FLAG_TCIF2_6))
  {
    __HAL_DMA_CLEAR_FLAG(&hdma_usart2_tx,DMA_FLAG_TCIF2_6);//__HAL_DMA_GET_IT_SOURCE
    HAL_UART_DMAStop(&hdma_usart2_tx);                     
  }
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
#endif
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

void DMA1_Stream6_IRQHandler(void)
{
/* USER CODE END DMA1_Stream6_IRQn 0 */
#if 0
  if(__HAL_DMA_GET_FLAG(&hdma_usart2_tx,DMA_FLAG_TCIF2_6))
  {
    __HAL_DMA_CLEAR_FLAG(&hdma_usart2_tx,DMA_FLAG_TCIF2_6);//__HAL_DMA_GET_IT_SOURCE
    HAL_UART_DMAStop(&hdma_usart2_tx);                     
  }
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
#endif
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  if (RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    Uart3RxFifo.in = GPS_BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
  }

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream7 global interrupt.
*/
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_tx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}
extern u8 CloudRxLen;
/**
* @brief This function handles UART5 global interrupt.
*/
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
  if (RESET != __HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    Uart5RxFifo.in = GPS_BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);
  }
  if (RESET != __HAL_UART_GET_FLAG(&huart5, UART_FLAG_FE))
  {
    HAL_UART_DMAStop(&huart5); //
    HAL_UART_Receive_DMA(&huart5, IMUUartBuf, IMU_BUFF_SIZE);
  }
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

extern uint8_t BlueTooth[GPS_BUFF_SIZE];
extern uint8_t data_to_bt[GPS_BUFF_SIZE];
int test_in = 0;
int test_out = 0;
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  uint32_t temp;

  if (RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    Uart2RxFifo.in = GPS_BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    test_in = Uart2RxFifo.in;
    test_out = Uart2RxFifo.out;
  }

  /* USER CODE END USART3_IRQn 0 */
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);

  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  uint32_t temp = 0;
  /* USER CODE BEGIN USART1_IRQn 0 */
  if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    Uart1RxFifo.in = IMU_BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
  }
  if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE))
  {
    HAL_UART_DMAStop(&huart1); //
    HAL_UART_Receive_DMA(&huart1, IMUUartBuf, IMU_BUFF_SIZE);
  }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}