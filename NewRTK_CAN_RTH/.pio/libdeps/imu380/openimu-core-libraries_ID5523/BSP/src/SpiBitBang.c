#include "SpiBitBang.h"
#include "bsp.h"
// #include "boardAPI.h"

int clockCycle = SPI_MIN_SPEED;

uint32_t spiNssMask = 0x000000E0; //all 3 pins
uint32_t spiNssSaveMask;
int saved = 0;


void _SpiDataOut(uint8_t *data, int len)
{
    uint8_t byte, level;
    uint8_t mask = 0x80;
    static uint8_t m  = 0;
    for (int i = 0; i < len; i++)
    {
        byte = *data++;
        mask = 0x80;
        for (int j = 0; j < 8; j++)
        {
            level = mask & byte;
            if (level)
            {
                // GPIOB->BSRRL = (uint32_t)SPI_MOSI_PIN;
                HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_SET);

            }
            else
            {
                //GPIOB->BSRRH = (uint32_t)SPI_MOSI_PIN;
                HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_RESET);

                
            }
            // clk low
            // GPIOB->BSRRH = (uint32_t)SPI_SCK_PIN;
            HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_RESET);

            //            IO3_PORT->BSRRH = IO3_PIN;
            for ( m = 0; m < clockCycle; m++)
                ;
            // clk high
            // GPIOB->BSRRL = (uint32_t)SPI_SCK_PIN;
            HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_SET);
            //            IO3_PORT->BSRRL = IO3_PIN;
            mask >>= 1;
            for ( m = 0; m < clockCycle; m++)
                ;
        }
    }
  
}

void _SpiDataIn(uint8_t *data, int len)
{
    static uint8_t m = 0;
    for (int j = 0; j < len; j++)
    {
        for (int i = 0; i < 8; i++)
        {
            // clk low
            // GPIOB->BSRRH = (uint32_t)SPI_SCK_PIN;
            HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_RESET);

            //            IO3_PORT->BSRRH = IO3_PIN;
            for ( m = 0; m < clockCycle; m++)
                ;
            // clk high
            // GPIOB->BSRRL = (uint32_t)SPI_SCK_PIN;
            HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_SET);

            //            IO3_PORT->BSRRL = IO3_PIN;
            *data++ = GPIOB->IDR & SPI_MISO_PINS;
            for (m = 0; m < clockCycle; m++)
                ;
        }
    }
   
}

void _SpiChipSelect()
{
    HAL_GPIO_WritePin(SPI_NSS_PORT, ((SPI_NSS_PINS)&spiNssMask), GPIO_PIN_RESET);
}

void _SpiChipDeSelect()
{
    HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PINS, GPIO_PIN_SET);
}

void _SpiSetStartCondition()
{
    // CLK
    if (START_CLK_LEVEL)
    {
        HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_RESET);
    }
    // data MOSI
    if (START_DATA_LEVEL)
    {
        HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_RESET);
    }
    // CS
    HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PINS, GPIO_PIN_SET);
}

uint8_t tmpBuf[1000];

void _SpiParseData(uint8_t *dst1, uint8_t *dst2, uint8_t *dst3, int len)
{
    uint8_t byte1, byte2, byte3;
    uint8_t *ptr = tmpBuf;

    for (int i = 0; i < len; i++)
    {
        byte1 = byte2 = byte3 = 0;
        for (int j = 0; j < 8; j++)
        {
            byte1 <<= 1;
            byte2 <<= 1;
            byte3 <<= 1;
            if (*ptr & SPI_MISO1_PIN)
            {
                byte1 |= 1;
            }
            if (*ptr & SPI_MISO2_PIN)
            {
                byte2 |= 1;
            }
            if (*ptr & SPI_MISO3_PIN)
            {
                byte3 |= 1;
            }
            ptr++;
        }
        *dst1++ = byte1;
        *dst2++ = byte2;
        *dst3++ = byte3;
    }
}

void SpiBitBangReadTransaction(uint8_t reg, uint8_t *dst1, uint8_t *dst2, uint8_t *dst3, int len)
{
    _SpiChipSelect();
    _SpiDataOut(&reg, 1);
    _SpiDataIn(tmpBuf, len);
    _SpiSetStartCondition();
    _SpiParseData(dst1, dst2, dst3, len);
}

void SpiBitBangWriteTransaction(uint8_t reg, uint8_t *src, int len)
{
    _SpiChipSelect();
    _SpiDataOut(&reg, 1);
    _SpiDataOut(src, len);
    _SpiSetStartCondition();
}

void SpiBitBangSetBaud(uint16_t prescaler)
{
    clockCycle = prescaler;
}

void SpiBitBangInit()
{
    board_SpiBitBang_Init();
    _SpiSetStartCondition();
}

void SpiBitBangSelectActiveSensors(uint16_t sensorsMap)
{
    spiNssMask = sensorsMap;
}

void SpiBitBangSelectSaveActiveSensors(uint16_t sensorsMap)
{
    spiNssSaveMask = spiNssMask;
    spiNssMask = (sensorsMap<<0x05);
    saved = 1;
}

void SpiBitBangRestoreActiveSensors()
{
    if (saved)
    {
        spiNssMask = spiNssSaveMask;
        saved = 0;
    }
}
