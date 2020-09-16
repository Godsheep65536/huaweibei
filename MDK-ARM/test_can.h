#ifndef _TEST__CAN_H
#define _TEST__CAN_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef CAN1_Send_Msg(uint32_t StdId, uint8_t *msg);
HAL_StatusTypeDef CAN1_Receive_Msg(uint8_t *buf);
void CanFilter_Init(CAN_HandleTypeDef* hcan);

#endif
