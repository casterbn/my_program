/** ***************************************************************************
 * @file bsp.h Board Support package, configure the Cortex M3
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#ifndef _BSP_GPIO_H
#define _BSP_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

//#include "timer.h"
#include "GlobalConstants.h"
// #include "boardAPI.h"
#include "stdint.h"

#define SPI_MOSI_PIN                       GPIO_PIN_0
#define SPI_SCK_PIN                        GPIO_PIN_1
#define SPI_MOSI_PORT                      GPIOB
#define SPI_SCK_PORT                       GPIOB

#define SPI_MISO1_PIN                      GPIO_PIN_2
#define SPI_MISO2_PIN                      GPIO_PIN_3
#define SPI_MISO3_PIN                      GPIO_PIN_4
#define SPI_MISO_PINS                      (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4)
#define SPI_MISO_PORT                      GPIOB

#define SPI_NSS1_PIN                       GPIO_PIN_5
#define SPI_NSS2_PIN                       GPIO_PIN_6
#define SPI_NSS3_PIN                       GPIO_PIN_7
#define SPI_NSS_PINS                       (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define SPI_NSS_PORT                       GPIOB

#define SPI2_PORT_CLK_ENABLE()             RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)

void BSP_init(void);
void RCC_config(void);
void GPIO_config(void);
void NVIC_config(void);
void DelayMs(uint32_t dly_ticks);
void set_DebugI2CPin(BOOL High);
void board_SpiBitBang_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* CONF_GPIO_H */

