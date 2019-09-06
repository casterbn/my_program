#ifndef __KSZ8041NL_H
#define __KSZ8041NL_H

#include "stm32f4xx_hal.h"

extern ETH_HandleTypeDef ETH_Handler;           
extern ETH_DMADescTypeDef *DMARxDscrTab;		
extern ETH_DMADescTypeDef *DMATxDscrTab;		
extern uint8_t *Rx_Buff; 						
extern uint8_t *Tx_Buff; 						
extern ETH_DMADescTypeDef  *DMATxDescToSet;		
extern ETH_DMADescTypeDef  *DMARxDescToGet; 	
 

uint8_t KSZ8041NL_Init(void);
uint32_t KSZ8041NL_ReadPHY(uint16_t reg);
void KSZ8041NL_WritePHY(uint16_t reg, uint16_t value);

uint8_t ETH_MACDMA_Config(void);
uint8_t ETH_Mem_Malloc(void);
void ETH_Mem_Free(void);
uint32_t  ETH_GetRxPktSize(ETH_DMADescTypeDef *DMARxDesc);

#endif
