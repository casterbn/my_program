/**
  ******************************************************************************
  * @file    bootMode.h
  ******************************************************************************
*/

#ifndef __BOOTMODE_H
#define __BOOTMODE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

//*****************************************************************************   
// Bitbang SPI definitions 
//*****************************************************************************   

#define IO3_PIN                            GPIO_PIN_11
#define IO3_PORT                           GPIOA
#define IO2_PIN                            GPIO_PIN_12
#define IO2_PORT                           GPIOA
#define IO3_CLK_ENABLE()                   __HAL_RCC_GPIOA_CLK_ENABLE()  
#define IO2_CLK_ENABLE()                   __HAL_RCC_GPIOA_CLK_ENABLE()  
#define GPIOA_CLK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE()  
#define GPIOA_CLK_DISABLE()                __HAL_RCC_GPIOA_CLK_DISABLE()  
#define GPIOB_CLK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()  

#define SPI_MISO1_PIN                      GPIO_PIN_3
#define SPI_MISO2_PIN                      GPIO_PIN_4
#define SPI_MISO3_PIN                      GPIO_PIN_6
#define SPI_MISO_PINS                      (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6)
#define SPI_MISO_PORT                      GPIOA

#define SPI_MOSI_PIN                       GPIO_PIN_7
#define SPI_SCK_PIN                        GPIO_PIN_5
#define SPI_MOSI_PORT                      GPIOA
#define SPI_SCK_PORT                       GPIOA

#define SPI_DRDY_PIN                       GPIO_PIN_5
#define SPI_DRDY_PORT                      GPIOB
#define DRDY_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()  


#define SPI_NSS1_PIN                       GPIO_PIN_0
#define SPI_NSS2_PIN                       GPIO_PIN_1
#define SPI_NSS3_PIN                       GPIO_PIN_2
#define SPI_NSS_PINS                       GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
#define SPI_NSS_PORT                       GPIOB

#define SPI_CLK_ENABLE()                  do{__HAL_RCC_GPIOA_CLK_ENABLE(); \
                                             __HAL_RCC_GPIOB_CLK_ENABLE();} while(0)  


// leds

#define LED1_PIN                            GPIO_PIN_8
#define LED1_PORT                           GPIOB
#define LED2_PIN                            GPIO_PIN_9
#define LED2_PORT                           GPIOB
#define LED3_PIN                            GPIO_PIN_6
#define LED3_PORT                           GPIOA
#define LED1_CLK_ENABLE()                   __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED2_CLK_ENABLE()                   __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED3_CLK_ENABLE()                   __HAL_RCC_GPIOA_CLK_ENABLE()  

#define ST_MODE_PIN                          GPIO_PIN_5
#define ST_MODE_PORT                         GPIOA
#define ST_MODE_CLK_ENABLE()                __HAL_RCC_GPIOA_CLK_ENABLE()  

#define ST_RESET_PIN                        GPIO_PIN_10
#define ST_RESET_PORT                       GPIOC
#define ST_RESET_CLK_ENABLE()                __HAL_RCC_GPIOC_CLK_ENABLE()  

#define ST_STDBY_PIN                        GPIO_PIN_6
#define ST_STDBY_PORT                       GPIOC
#define ST_STDBY_CLK_ENABLE()                __HAL_RCC_GPIOC_CLK_ENABLE()  

#define ST_WKUP_PIN                         GPIO_PIN_2
#define ST_WKUP_PORT                        GPIOC
#define ST_WKUP_CLK_ENABLE()                __HAL_RCC_GPIOC_CLK_ENABLE()  

#define ST_BOOT_PIN                         GPIO_PIN_0
#define ST_BOOT_PORT                        GPIOD
#define ST_BOOT_CLK_ENABLE()                __HAL_RCC_GPIOD_CLK_ENABLE()  

#define ST_PROG_BUF_CTL_PIN                  GPIO_PIN_0
#define ST_PROG_BUF_CTL_PORT                 GPIOC
#define ST_PROG_BUF_CTL_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()  

#define ST_UART2_TX_PIN                      GPIO_PIN_10
#define ST_UART2_RX_PIN                      GPIO_PIN_11
#define ST_UART2_PORT                        GPIOB
#define ST_UART2_CLK_ENABLE()                __HAL_RCC_GPIOB_CLK_ENABLE()  


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART6
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART6_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART6_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART6_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_6
#define USARTx_TX_GPIO_PORT              GPIOC
#define USARTx_TX_AF                     GPIO_AF8_USART6
#define USARTx_RX_PIN                    GPIO_PIN_7
#define USARTx_RX_GPIO_PORT              GPIOC
#define USARTx_RX_AF                     GPIO_AF8_USART6

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_STREAM              DMA2_Stream7
#define USARTx_RX_DMA_STREAM              DMA2_Stream1
#define USARTx_TX_DMA_CHANNEL             DMA_CHANNEL_5
#define USARTx_RX_DMA_CHANNEL             DMA_CHANNEL_5


/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA2_Stream7_IRQn
#define USARTx_DMA_RX_IRQn                DMA2_Stream1_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA2_Stream7_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA2_Stream1_IRQHandler

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART6_IRQn
#define USARTx_IRQHandler                USART6_IRQHandler




void   IO3_Toggle();
void   IO2_Toggle();
void   IO2_On();
void   IO3_On();
void   IO2_Off();
void   IO3_Off();
void    DRDY_Off();
void    DRDY_On();

void   LED1_Toggle();
void   LED2_Toggle();
void   LED3_Toggle();
void   LED1_On();
void   LED2_On();
void   LED3_On();
void   LED1_Off();
void   LED2_Off();
void   LED3_Off();
void   BSP_Init_LEDS();
void   BSP_Init_ST_BOOT_MODE();
int    ST_BootREquired();
void   BSP_Reset_ST();
void   BSP_Init_ST_Interface_PINs();
void BSP_Init_ST_PASSTHROUIGH_MODE();





#ifdef __cplusplus
}
#endif

#endif

