/** ***************************************************************************
 * @file   configureGPIO.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * Initialize the GPIO pins to output for the 380 board
 ******************************************************************************/
#include "configureGPIO.h"
// #include "boardAPI.h"
#include "GlobalConstants.h"
// #include "platformAPI.h"
#include "stm32f4xx_hal_conf.h"

void DelayMs(uint32_t msec)
{
    for (int i = msec; i > 0; i--)
    {
        for (int j = 0; j < 30000000; j++)
            i--;
    }
}

/** ****************************************************************************
 * @name _InitCommonPins_GPIO LOCAL
 *
 * @brief Initialize the GPIO pins common to the UART and SPI3 communication
 *        ports as input pins (one will be configured an output based on the
 *        communications protocol selected)
 * Trace:
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void _InitCommonPins_GPIO(void)
{
    /// ---- Initialize UARTB RX and SPI3 nSS as input pins so, regardless of how
    ///      SPI is configured (in 'spi_configure'), neither interferes with the
    ///      other as they are tied together ----
    // InitPin_GPIO(SPI3_SLAVE_SELECT_CLK,
    //              SPI3_SLAVE_SELECT_PORT,
    //              SPI3_SLAVE_SELECT_PIN,
    //              GPIO_INPUT);

    // InitPin_GPIO(USER_B_UART_RX_GPIO_CLK,
    //              USER_B_UART_RX_GPIO_PORT,
    //              USER_B_UART_RX_PIN,
    //              GPIO_INPUT);
}

/** ****************************************************************************
 * @name InitUnitConfigurationPins LOCAL
 *
 * @brief Initialize the configuration pins
 * Trace:
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void _InitUnitConfigurationPins(void)
{
    // InitPin_GPIO(CONFIG_0_CLK, CONFIG_0_PORT, CONFIG_0_PIN, GPIO_INPUT);
    // InitPin_GPIO(CONFIG_1_CLK, CONFIG_1_PORT, CONFIG_1_PIN, GPIO_INPUT);
    // InitPin_GPIO(CONFIG_2_CLK, CONFIG_2_PORT, CONFIG_2_PIN, GPIO_INPUT);
}

/** ****************************************************************************
 * @name InitBoardConfiguration_GPIO
 *
 * @brief Initialize the pins for the 380 board called in main()
 * Trace:
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void InitBoardConfiguration_GPIO()
{
    /// If pulled low at power up, the pin will be used as an input whose signal
    ///   syncs the clock to the GPS 1PPS signal. If not pulled low, the pin
    ///   will provide a 1PPS (or other frequency) signal.
    // InitPin_GPIO(ONE_PPS_CLK,
    //              ONE_PPS_PORT,
    //              ONE_PPS_PIN,
    //              GPIO_INPUT);

    /// Initialize the nSS and UARTB RX pins as input-pins (they share a common
    ///    line).  Change the status of the pins in taskUserCommunication()
    ///    depending on which communication mode is selected.
    _InitCommonPins_GPIO();

 
}

/** ****************************************************************************
 * @name ReadUnitConfiguration_GPIO
 *
 * @brief Initialize the pins for the 380 board called in main()
 * Trace:
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
uint8_t ReadUnitHwConfiguration(void)
{
    return 0;
}

/** ****************************************************************************
 * @name InitPin_GPIO
 *
 * @brief Initialize a GPIO pin (using peripheral clock AHB1) as an input/output
 *        pin
 * Trace:
 *
 * @param [In] PeriphClock clock ID
 * @param [In] GPIO_Port GPIO port
 * @param [In] GPIO_Pin  GPIO Pin
 * @param [In] inputOutputSelector - input or output
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void InitPin_GPIO(uint32_t PeriphClock,
                  GPIO_TypeDef *GPIO_Port,
                  uint32_t GPIO_Pin,
                  uint8_t inputOutputSelector)
{
  

 
}

// void ResetSTIForNormalMode()
// {
//     HAL_GPIO_WritePin(GPIOC,STI_RESET_PIN,GPIO_PIN_RESET);
//     // GPIOC->BSRRH = STI_RESET_PIN; // Set nRST low
//     HAL_GPIO_WritePin(GPIOB,STI_BOOT_PIN,GPIO_PIN_RESET);
//     // GPIOB->BSRRH = STI_BOOT_PIN;  // Set BOOT pin low
//     DelayMs(10);
//     HAL_GPIO_WritePin(GPIOC,STI_RESET_PIN,GPIO_PIN_SET);
//     // GPIOC->BSRRL = STI_RESET_PIN; // Set nRST high
//     DelayMs(10);
// }

void SetBufCtr(uint8_t state)
{
    if(state)
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    else 
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    

}

void ResetSTIForBootMode()
{

    // SetBufCtr(GPIO_PIN_SET);
    SetBufCtr(GPIO_PIN_RESET);
    // GPIOC->BSRRH = STI_RESET_PIN; // Set nRST low
    HAL_GPIO_WritePin(GPIOC,STI_RESET_PIN,GPIO_PIN_RESET);
    DelayMs(10);
    HAL_GPIO_WritePin(GPIOD,STI_BOOT_PIN,GPIO_PIN_SET);
    // GPIOB->BSRRL = STI_BOOT_PIN;  // Set BOOT pin high
    DelayMs(10);
    // GPIOC->BSRRL = STI_RESET_PIN; // Set nRST high
    HAL_GPIO_WritePin(GPIOC,STI_RESET_PIN,GPIO_PIN_SET);
    DelayMs(10);
}

void ConfigureModePinAsOutput()
{
    // InitPin_GPIO(RCC_AHB1Periph_GPIOC, GPIOC, STI_MODE_PIN, GPIO_OUTPUT);
}

void SetModePin(int level)
{
    // if (level)
    // {
    //     GPIOC->BSRRL = STI_MODE_PIN; // Set high
    // }
    // else
    // {
    //     GPIOC->BSRRH = STI_MODE_PIN; // Set low
    // }
}

uint8_t isSTIBootMode()
{
    return ((GPIOD->IDR & STI_MODE_PIN) == 0);
}
