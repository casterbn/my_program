
/****************************************       DMA      ************************************/


/* 私有类型定义 --------------------------------------------------------------*/
typedef enum 
{
  FAILED = 0,
  PASSED = !FAILED
} TestStatus;

/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
DMA_HandleTypeDef hdma_memtomem_dma1_channel1;

static const uint32_t SRC_Const_Buffer[24]= {
  0x01,0x02,0x03,0x04,
  0x05,0x06,0x07,0x08,
  0x09,0x0a,0x0b,0x0c,
  0x0d,0x0e,0x0f,0x10,
  0x11,0x12,0x13,0x14,
  0x15,0x16,0x17,0x18
  };

uint32_t DST_Buffer[32];
__IO TestStatus TransferStatus= FAILED;
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
TestStatus Buffercmp(const uint32_t* pBuffer, uint32_t* pBuffer1, uint16_t BufferLength);
void MX_DMA_Init(void);

/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{      
  HAL_StatusTypeDef har_status;
  int test = 0;
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();
  
  /* 板载LED初始化 */
  //LED_GPIO_Init();
  
  /* DMA初始化 */
  MX_DMA_Init_test();
  MX_USART1_UART_Init();

  //har_status=HAL_DMA_Start(&hdma_memtomem_dma1_channel1,(uint32_t)&SRC_Const_Buffer,(uint32_t)&DST_Buffer,32);
  memcpy_aceinna((uint32_t)&SRC_Const_Buffer,(uint32_t)&DST_Buffer,24);
  if(har_status==HAL_OK)
  {
    /* 检查发送和接收的数据是否相等 */

    TransferStatus = Buffercmp(SRC_Const_Buffer, DST_Buffer, 24);
    //HAL_UART_Transmit(&huart1, DST_Buffer, 32, 0xFFFF);
    /* 如果接收和发送的数据都是相同的，则通过 */
    if(TransferStatus == PASSED)
    {
      //LED1_ON;
        test++;
    }
    /* 如果接收和发送的数据不同，则传输出错 */
    else
    {
      //LED2_ON;
    }
  }
  else
  {
    //LED3_ON;
  }
  /* 无限循环 */
  while (1)
  {
    //TransferStatus = Buffercmp(SRC_Const_Buffer, DST_Buffer, 32);
    delay_ms(2000);
    //HAL_UART_Transmit(&huart1, SRC_Const_Buffer, 32, 0xFFFF);
    //HAL_UART_Transmit(&huart1, DST_Buffer, 32, 0xFFFF);
  }
}

/**
  * 函数功能: DMA配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void MX_DMA_Init_test(void) 
{
  /* 使能DMA控制器时钟 */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  /* 配置DMA通道工作方式 */
  hdma_memtomem_dma1_channel1.Instance = DMA2_Stream0;
      hdma_memtomem_dma1_channel1.Init.Channel = DMA_CHANNEL_1;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_memtomem_dma1_channel1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  
  HAL_DMA_Init(&hdma_memtomem_dma1_channel1);



/*
    hdma_uart5_rx.Instance = DMA1_Stream0;
    hdma_uart5_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

*/
}

/**
  * 函数功能: 判断指定长度的两个数据源是否完全相等
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 如果完全相等返回1，只要其中一对数据不相等返回0
  */
TestStatus Buffercmp(const uint32_t* pBuffer, uint32_t* pBuffer1, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer != *pBuffer1)
    {
      return FAILED;
    }
    
    pBuffer++;
    pBuffer1++;
  }
  return PASSED;  
}

void memcpy_aceinna(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
    int har_status=HAL_DMA_Start(&hdma_memtomem_dma1_channel1,SrcAddress,DstAddress,DataLength);
}
