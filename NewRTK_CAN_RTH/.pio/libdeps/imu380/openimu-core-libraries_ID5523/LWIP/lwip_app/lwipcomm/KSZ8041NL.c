#include "KSZ8041NL.h"
#include "lwipcomm.h"
#include "osapi.h"
#include "malloc.h"
#include "configureGPIO.h"

ETH_HandleTypeDef ETH_Handler;

ETH_DMADescTypeDef *DMARxDscrTab;
ETH_DMADescTypeDef *DMATxDscrTab;
uint8_t *Rx_Buff;
uint8_t *Tx_Buff;

uint8_t KSZ8041NL_Init(void)
{
    uint8_t macaddress[6];
    GPIO_InitTypeDef GPIO_Initure;

    // PE0
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_Initure.Pin = GPIO_PIN_0;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_Initure);

    // close all interupt
    // reset
    __set_PRIMASK(1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    OS_Delay(500);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
    OS_Delay(1000);
    OS_Delay(1000);
    __set_PRIMASK(0);
    // open all interupt

    macaddress[0] = lwipdev.mac[0];
    macaddress[1] = lwipdev.mac[1];
    macaddress[2] = lwipdev.mac[2];
    macaddress[3] = lwipdev.mac[3];
    macaddress[4] = lwipdev.mac[4];
    macaddress[5] = lwipdev.mac[5];

    ETH_Handler.Instance = ETH;
    ETH_Handler.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
    ETH_Handler.Init.Speed = ETH_SPEED_100M;
    ETH_Handler.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
    ETH_Handler.Init.PhyAddress = KSZ8041NL_PHY_ADDRESS;
    ETH_Handler.Init.MACAddr = macaddress;
    ETH_Handler.Init.RxMode = ETH_RXINTERRUPT_MODE;
    ETH_Handler.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
    ETH_Handler.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
    if (HAL_ETH_Init(&ETH_Handler) == HAL_OK)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_ETH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* RMII
    ETH_MDIO -------------------------> PA2    
    ETH_MDC --------------------------> PC1    
    ETH_RMII_REF_CLK------------------> PA1    
    ETH_RMII_CRS_DV ------------------> PA7    
    ETH_RMII_RXD0 --------------------> PC4    
    ETH_RMII_RXD1 --------------------> PC5    
    ETH_RMII_TX_EN -------------------> PG11   
    ETH_RMII_TXD0 --------------------> PG13   
    ETH_RMII_TXD1 --------------------> PG14   
    ETH_RESET-------------------------> PE0 */

    //PA1,2,7
    GPIO_Initure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
    GPIO_Initure.Mode = GPIO_MODE_AF_PP;
    GPIO_Initure.Pull = GPIO_NOPULL;
    GPIO_Initure.Speed = GPIO_SPEED_HIGH;
    GPIO_Initure.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);

    //PG11
    GPIO_Initure.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOG, &GPIO_Initure);

    //PC1,4,5
    GPIO_Initure.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &GPIO_Initure);

    //PG13,14
    GPIO_Initure.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(GPIOG, &GPIO_Initure);

    HAL_NVIC_SetPriority(ETH_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

uint32_t KSZ8041NL_ReadPHY(uint16_t reg)
{
    uint32_t regval;
    HAL_ETH_ReadPHYRegister(&ETH_Handler, reg, &regval);
    return regval;
}

void KSZ8041NL_WritePHY(uint16_t reg, uint16_t value)
{
    uint32_t temp = value;
    HAL_ETH_ReadPHYRegister(&ETH_Handler, reg, &temp);
}

extern void lwip_pkt_handle(void); // lwip_comm.c

void ETH_IRQHandler(void)       //ETH接收中断
{
    while (ETH_GetRxPktSize(ETH_Handler.RxDesc))
    {
        lwip_pkt_handle(); // push data to LWIP 调用low_level_input
    }

    __HAL_ETH_DMA_CLEAR_IT(&ETH_Handler, ETH_DMA_IT_NIS);
    __HAL_ETH_DMA_CLEAR_IT(&ETH_Handler, ETH_DMA_IT_R);
}

uint32_t ETH_GetRxPktSize(ETH_DMADescTypeDef *DMARxDesc)
{
    uint32_t frameLength = 0;
    if (((DMARxDesc->Status & ETH_DMARXDESC_OWN) == (uint32_t)RESET) &&
        ((DMARxDesc->Status & ETH_DMARXDESC_ES) == (uint32_t)RESET) &&
        ((DMARxDesc->Status & ETH_DMARXDESC_LS) != (uint32_t)RESET))
    {
        frameLength = ((DMARxDesc->Status & ETH_DMARXDESC_FL) >> ETH_DMARXDESC_FRAME_LENGTHSHIFT);
    }
    return frameLength;
}

/** ***************************************************************************
 * @name ETH_Mem_Malloc() malloc memory 
 * @brief malloc memory for ETH low driver. 
 * 	if malloc failure, ETH_Mem_Free() will be called
 * @param [in] N/A
 * @retval 0 or 1 indicating success or failure of malloc memory 
 ******************************************************************************/
uint8_t ETH_Mem_Malloc(void)
{
    DMARxDscrTab = mymalloc(SRAMIN, ETH_RXBUFNB * sizeof(ETH_DMADescTypeDef));
    DMATxDscrTab = mymalloc(SRAMIN, ETH_TXBUFNB * sizeof(ETH_DMADescTypeDef));
    Rx_Buff = mymalloc(SRAMIN, ETH_RX_BUF_SIZE * ETH_RXBUFNB);
    Tx_Buff = mymalloc(SRAMIN, ETH_TX_BUF_SIZE * ETH_TXBUFNB);

    if (!(uint32_t)&DMARxDscrTab || !(uint32_t)&DMATxDscrTab || !(uint32_t)&Rx_Buff || !(uint32_t)&Tx_Buff)
    {
        ETH_Mem_Free();
        return 1; // failure
    }
    return 0; // success
}

/** ***************************************************************************
 * @name ETH_Mem_Free() free memory 
 * @brief free memory of ETH low driver
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void ETH_Mem_Free(void)
{
    myfree(SRAMIN, DMARxDscrTab);
    myfree(SRAMIN, DMATxDscrTab);
    myfree(SRAMIN, Rx_Buff);
    myfree(SRAMIN, Tx_Buff);
}
