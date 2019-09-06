/**
  bootMode.c
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "bootMode.h"

void IO3_On()
{
  HAL_GPIO_WritePin(IO3_PORT, IO3_PIN, GPIO_PIN_SET); 
}

void IO3_Off()
{
  HAL_GPIO_WritePin(IO3_PORT, IO3_PIN, GPIO_PIN_RESET); 
}

void IO2_On()
{
  HAL_GPIO_WritePin(IO2_PORT, IO2_PIN, GPIO_PIN_SET); 
}

void IO2_Off()
{
  HAL_GPIO_WritePin(IO2_PORT, IO2_PIN, GPIO_PIN_RESET); 
}


void IO3_Toggle()
{
  HAL_GPIO_TogglePin(IO3_PORT, IO3_PIN);
}

void IO2_Toggle()
{
  HAL_GPIO_TogglePin(IO2_PORT, IO2_PIN);
}


void DRDY_Off()
{
  HAL_GPIO_WritePin(SPI_DRDY_PORT, SPI_DRDY_PIN, GPIO_PIN_SET); 
}

void DRDY_On()
{
  HAL_GPIO_WritePin(SPI_DRDY_PORT, SPI_DRDY_PIN, GPIO_PIN_RESET); 
}


void LED1_On()
{
  HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET); 
}

void LED1_Off()
{
  HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET); 
}

void LED2_On()
{
  HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET); 
}

void LED2_Off()
{
  HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET); 
}
void LED3_On()
{
  HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_SET); 
}

void LED3_Off()
{
  HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_RESET); 
}


void LED1_Toggle()
{
  HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
}

void LED2_Toggle()
{
  HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN);
}

void LED3_Toggle()
{
  HAL_GPIO_TogglePin(LED3_PORT, LED3_PIN);
}

int ST_BootREquired()
{
    GPIO_PinState state = HAL_GPIO_ReadPin(ST_MODE_PORT, ST_MODE_PIN);
    return (int)(state == 0);
}


void BSP_Init_LEDS()
{
  GPIO_InitTypeDef  gpio_init_structure;

    LED1_CLK_ENABLE();
    LED2_CLK_ENABLE();
    LED3_CLK_ENABLE();

  // Configure the GPIO_LED pins
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    // LED1
    gpio_init_structure.Pin   = LED1_PIN;
    HAL_GPIO_Init(LED1_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);

    // LED2
    gpio_init_structure.Pin   = LED2_PIN;
    HAL_GPIO_Init(LED2_PORT, &gpio_init_structure);
    // By default, turn off LED by setting a high level on corresponding GPIO
    HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);

    // LED3
    gpio_init_structure.Pin   = LED3_PIN;
    HAL_GPIO_Init(LED3_PORT, &gpio_init_structure);
    // By default, turn off LED by setting a high level on corresponding GPIO
    HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_SET);

}


void BSP_Init_ST_Interface_PINs()
{
    GPIO_InitTypeDef  gpio_init_structure;
    
    ST_MODE_CLK_ENABLE();
    ST_RESET_CLK_ENABLE();
    ST_STDBY_CLK_ENABLE();
    ST_WKUP_CLK_ENABLE();
    ST_BOOT_CLK_ENABLE();

// Configure the BOOT mode detect pin
    // gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    // gpio_init_structure.Pull  = GPIO_PULLUP;
    // gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    // gpio_init_structure.Pin   = ST_MODE_PIN;
    // HAL_GPIO_Init(ST_MODE_PORT, &gpio_init_structure);

// Configure the RESET pin
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_RESET_PIN;
    HAL_GPIO_Init(ST_RESET_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(ST_RESET_PORT, ST_RESET_PIN, GPIO_PIN_SET);

// Configure the STDBY pin
    gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_STDBY_PIN;
    HAL_GPIO_Init(ST_STDBY_PORT, &gpio_init_structure);

// Configure the WKUP pin
    gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_WKUP_PIN;
    HAL_GPIO_Init(ST_WKUP_PORT, &gpio_init_structure);

// Configure the BOOT pin for ST normal mode
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_BOOT_PIN;
    HAL_GPIO_Init(ST_BOOT_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(ST_BOOT_PORT, ST_BOOT_PIN, GPIO_PIN_RESET);

}

void BSP_Init_ST_BOOT_MODE()
{
    GPIO_InitTypeDef  gpio_init_structure;

    // Configure the ST UART2 MCU pins as inputs
    gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_UART2_TX_PIN;
    HAL_GPIO_Init(ST_UART2_PORT, &gpio_init_structure);
    gpio_init_structure.Pin   = ST_UART2_RX_PIN;
    HAL_GPIO_Init(ST_UART2_PORT, &gpio_init_structure);

    // Configure the ST UART BUF control pin and set it high for enabling buffer
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_PROG_BUF_CTL_PIN;
    HAL_GPIO_Init(ST_PROG_BUF_CTL_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(ST_PROG_BUF_CTL_PORT, ST_PROG_BUF_CTL_PIN, GPIO_PIN_SET);
    // Set BOOT pin
    HAL_GPIO_WritePin(ST_BOOT_PORT, ST_BOOT_PIN, GPIO_PIN_SET);
}

void BSP_Init_ST_PASSTHROUIGH_MODE()
{
    GPIO_InitTypeDef  gpio_init_structure;

    // Configure the ST UART2 MCU pins as inputs
    gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_UART2_TX_PIN;
    HAL_GPIO_Init(ST_UART2_PORT, &gpio_init_structure);
    gpio_init_structure.Pin   = ST_UART2_RX_PIN;
    HAL_GPIO_Init(ST_UART2_PORT, &gpio_init_structure);

    // Configure the ST UART BUF control pin and set it high for enabling buffer
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_PROG_BUF_CTL_PIN;
    HAL_GPIO_Init(ST_PROG_BUF_CTL_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(ST_PROG_BUF_CTL_PORT, ST_PROG_BUF_CTL_PIN, GPIO_PIN_SET);
    // Set BOOT pin
    HAL_GPIO_WritePin(ST_BOOT_PORT, ST_BOOT_PIN, GPIO_PIN_RESET);
}



void BSP_Reset_ST()
{
     HAL_GPIO_WritePin(ST_RESET_PORT, ST_RESET_PIN, GPIO_PIN_RESET);
     DelayMs(100);
     HAL_GPIO_WritePin(ST_RESET_PORT, ST_RESET_PIN, GPIO_PIN_SET);
     DelayMs(100);
}


