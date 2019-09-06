#include "stm32f4xx_hal.h"
#include "osapi.h"
#include "osresources.h"

#include "stdio.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "taskUserSpi.h"
#include "rtcm.h"
#include "main.h"

/* master for test */
//#define USER_SPI_MASTER

extern UART_HandleTypeDef huart1;

int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}

SPI_HandleTypeDef SPI5_Handler;

DMA_HandleTypeDef SPI5_dma_tx;
DMA_HandleTypeDef SPI5_dma_rx;

uint8_t SPI5_txBuf[128];
uint8_t SPI5_rxBuf[128];

/*  PF6     MCU_USER_NSS    SPI5_NSS
 *  PF7     MCU_USER_SCK    SPI5_SCK
 *  PF8     MCU_USER_MISO   SPI5_MISO
 *  PF9     MCU_USER_MOSI   SPI5_MOSI
 *  PB12    MCU_USER_DRDY  
 */
void userSpiInit(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOF_CLK_ENABLE();

#ifdef USER_SPI_MASTER
    GPIO_Initure.Pin = GPIO_PIN_6;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOF, &GPIO_Initure);

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
#else
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_Initure.Pin = GPIO_PIN_12;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);

    GPIO_Initure.Pin = GPIO_PIN_6;
    //GPIO_Initure.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_Initure.Mode = GPIO_MODE_IT_RISING;    //TODO:
    GPIO_Initure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOF, &GPIO_Initure);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif

    SPI5_Handler.Instance = SPI5;
    SPI5_Handler.Init.Direction = SPI_DIRECTION_2LINES;             // 双线模式
    SPI5_Handler.Init.DataSize = SPI_DATASIZE_8BIT;                 // 8位帧结构
    SPI5_Handler.Init.CLKPolarity = SPI_POLARITY_HIGH;              // 同步时钟的空闲状态为高电平
    SPI5_Handler.Init.CLKPhase = SPI_PHASE_2EDGE;                   // 同步时钟的第二个边沿数据被采样
    SPI5_Handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // 波特率预分频值为32
    SPI5_Handler.Init.FirstBit = SPI_FIRSTBIT_MSB;                  // 数据传输从MSB位开始
    SPI5_Handler.Init.TIMode = SPI_TIMODE_DISABLE;                  // 关闭TI模式
    SPI5_Handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;  // 关闭硬件CRC校验
    SPI5_Handler.Init.CRCPolynomial = 7;                            // CRC值计算的多项式
#ifdef USER_SPI_MASTER
    SPI5_Handler.Init.Mode = SPI_MODE_MASTER;
    SPI5_Handler.Init.NSS = SPI_NSS_SOFT;
#else
    SPI5_Handler.Init.Mode = SPI_MODE_SLAVE;
    SPI5_Handler.Init.NSS = SPI_NSS_HARD_INPUT; // SPI_NSS_HARD_INPUT
#endif
    
    if (HAL_SPI_Init(&SPI5_Handler) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI5)
    {
        GPIO_InitTypeDef GPIO_Initure;

        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_SPI5_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        GPIO_Initure.Pin = GPIO_PIN_7;
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_HIGH;
        GPIO_Initure.Alternate = GPIO_AF5_SPI5;
        HAL_GPIO_Init(GPIOF, &GPIO_Initure);

        GPIO_Initure.Pin = GPIO_PIN_8;
        GPIO_Initure.Alternate = GPIO_AF5_SPI5;
        HAL_GPIO_Init(GPIOF, &GPIO_Initure);

        GPIO_Initure.Pin = GPIO_PIN_9;
        GPIO_Initure.Alternate = GPIO_AF5_SPI5;
        HAL_GPIO_Init(GPIOF, &GPIO_Initure);

        SPI5_dma_tx.Instance = DMA2_Stream4;
        SPI5_dma_tx.Init.Channel = DMA_CHANNEL_2;
        SPI5_dma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        SPI5_dma_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        SPI5_dma_tx.Init.MemBurst = DMA_MBURST_INC4;
        SPI5_dma_tx.Init.PeriphBurst = DMA_PBURST_INC4;
        SPI5_dma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        SPI5_dma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        SPI5_dma_tx.Init.MemInc = DMA_MINC_ENABLE;
        SPI5_dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        SPI5_dma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        SPI5_dma_tx.Init.Mode = DMA_NORMAL;
        SPI5_dma_tx.Init.Priority = DMA_PRIORITY_LOW;

        HAL_DMA_Init(&SPI5_dma_tx);
        __HAL_LINKDMA(hspi, hdmatx, SPI5_dma_tx);

        SPI5_dma_rx.Instance = DMA2_Stream3;
        SPI5_dma_rx.Init.Channel = DMA_CHANNEL_2;
        SPI5_dma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        SPI5_dma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        SPI5_dma_rx.Init.MemBurst = DMA_MBURST_INC4;
        SPI5_dma_rx.Init.PeriphBurst = DMA_PBURST_INC4;
        SPI5_dma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        SPI5_dma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        SPI5_dma_rx.Init.MemInc = DMA_MINC_ENABLE;
        SPI5_dma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        SPI5_dma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        SPI5_dma_rx.Init.Mode = DMA_NORMAL; // DMA_CIRCULAR
        SPI5_dma_rx.Init.Priority = DMA_PRIORITY_HIGH;

        HAL_DMA_Init(&SPI5_dma_rx);
        __HAL_LINKDMA(hspi, hdmarx, SPI5_dma_rx);

        HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

        HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

        HAL_NVIC_SetPriority(SPI5_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(SPI5_IRQn);
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI5)
    {
        __HAL_RCC_SPI5_FORCE_RESET();
        __HAL_RCC_SPI5_RELEASE_RESET();

        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_7);
        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_8);
        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_9);

        HAL_DMA_DeInit(&SPI5_dma_tx);
        HAL_DMA_DeInit(&SPI5_dma_rx);

        HAL_NVIC_DisableIRQ(DMA2_Stream4_IRQn);
        HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);

        HAL_NVIC_DisableIRQ(SPI5_IRQn);
    }
}

// MCU_USER_NSS
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    printf("IRQ\n");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_6)
    {
        reset_ms();
        if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6) == GPIO_PIN_RESET)
        {
            //printf("s fall\r\n");
        }
        else
        {
            //printf("s rise\r\n");
        }
    }
}

void DMA2_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(SPI5_Handler.hdmatx);
}

void DMA2_Stream3_IRQHandler(void)
{
    // if (__HAL_DMA_GET_FLAG(SPI5_Handler.hdmarx, DMA_FLAG_TCIF3_7))
    // {
    //     printf("hdmarxTCIF\r\n");
    // }
    // else if (__HAL_DMA_GET_FLAG(SPI5_Handler.hdmarx, DMA_FLAG_HTIF3_7))
    // {
    //     printf("hdmarxHTIF\r\n");
    // }

    HAL_DMA_IRQHandler(SPI5_Handler.hdmarx);
}

void SPI5_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&SPI5_Handler);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
#ifdef USER_SPI_MASTER
    printf("m txrx: %02x %02x %02x %02x\r\n", SPI5_rxBuf[0], SPI5_rxBuf[1], SPI5_rxBuf[2], SPI5_rxBuf[3]);
    memset(SPI5_rxBuf, 0, sizeof(SPI5_rxBuf));
#else
    printf("s txrx: %02x %02x %02x %02x\r\n", SPI5_rxBuf[0], SPI5_rxBuf[1], SPI5_rxBuf[2], SPI5_rxBuf[3]);
    memset(SPI5_rxBuf, 0, sizeof(SPI5_rxBuf));
    HAL_SPI_TransmitReceive_DMA(&SPI5_Handler, SPI5_txBuf, SPI5_rxBuf, 4);
#endif
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    printf("s rx: %02x %02x %02x %02x\r\n", SPI5_rxBuf[0], SPI5_rxBuf[1], SPI5_rxBuf[2], SPI5_rxBuf[3]);
    memset(SPI5_rxBuf, 0, sizeof(SPI5_rxBuf));
    HAL_SPI_Receive_DMA(&SPI5_Handler, (uint8_t *)SPI5_rxBuf, 4);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    printf("HAL_SPI_ErrorCallback\r\n");
    userSpiInit();
    HAL_SPI_TransmitReceive_DMA(&SPI5_Handler, (uint8_t *)SPI5_txBuf, (uint8_t *)SPI5_rxBuf, 4);
}

void userSpiWrite(uint8_t *txBuffer, uint8_t *rxBuffer, uint32_t len)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&SPI5_Handler, txBuffer, rxBuffer, len, 1000);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
}

void UserSpiTask(void const *argument)
{
    static int num = 0;

    userSpiInit();

#ifndef USER_SPI_MASTER
    SPI5_txBuf[0] = 0xE1;
    SPI5_txBuf[1] = 0xE2;
    SPI5_txBuf[2] = 0xE3;
    SPI5_txBuf[3] = 0xE4;
    HAL_SPI_TransmitReceive_DMA(&SPI5_Handler, SPI5_txBuf, SPI5_rxBuf, 4);
#endif

    while (1)
    {
        num++;
        if (num == 50)
        {
            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        }
        else if (num == 200)
        {
            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // GPIO_PIN_8
            num = 0;

#ifdef USER_SPI_MASTER
            SPI5_txBuf[0] = 0x01;
            SPI5_txBuf[1] = 0x02;
            SPI5_txBuf[2] = 0x03;
            SPI5_txBuf[3] = 0x04;
            //userSpiWrite(SPI5_txBuf, SPI5_rxBuf, 4);
            HAL_SPI_TransmitReceive_DMA(&SPI5_Handler, SPI5_txBuf, SPI5_rxBuf, 4);
            printf("send\r\n");
#endif
        }
        TIME_S *get_time_s;
        get_time_s = get_time();
        //printf("ms = %d\n",get_time_s->ms);
        OS_Delay(100);
    }
}
