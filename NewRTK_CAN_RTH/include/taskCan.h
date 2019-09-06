#ifndef _TASK_CAN_H_
#define _TASK_CAN_H_

void can_config(int baudrate);

uint8_t CAN_Send_Msg(void);
uint8_t CAN_Receive_Msg(void);

void CanTask(void const *argument);

#endif
