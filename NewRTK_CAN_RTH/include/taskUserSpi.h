#ifndef _TASK_USER_SPI_H_
#define _TASK_USER_SPI_H_

void userSpiInit(void);

void userSpiWrite(uint8_t *txBuffer, uint8_t *rxBuffer, uint32_t len);

void UserSpiTask(void const *argument);



#endif
