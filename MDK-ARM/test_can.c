#include "test_can.h"
#include "can.h"

CAN_RxHeaderTypeDef pRxMsg;
CAN_TxHeaderTypeDef pTxMsg;
uint8_t CAN1_buff[8];

void CanFilter_Init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterTypeDef canfilter;

  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
  canfilter.SlaveStartFilterBank=14;     //从can的过滤器起始编号 只有当设置两个can时 该参数才有意义
  if(hcan->Instance == CAN1)
  {
		canfilter.FilterBank = 0;              //主can的过滤器编号
		canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;// CAN_FilterFIFO0;
  }
	canfilter.FilterActivation = ENABLE;              //激活过滤器
  HAL_CAN_ConfigFilter(hcan, &canfilter);

}

HAL_StatusTypeDef CAN1_Send_Msg(uint32_t StdId, uint8_t *msg)
{
  pTxMsg.StdId = StdId;      //标准标识符
  pTxMsg.IDE = CAN_ID_STD;   //使用标准帧
  pTxMsg.RTR = CAN_RTR_DATA; //数据帧
  pTxMsg.DLC = 8;
	pTxMsg.TransmitGlobalTime = DISABLE;

	HAL_StatusTypeDef err =  HAL_CAN_AddTxMessage(&hcan1, &pTxMsg, msg, (uint32_t *)CAN_TX_MAILBOX0);
	return err;
}

inline HAL_StatusTypeDef CAN1_Receive_Msg(uint8_t *buf)
{
	return HAL_CAN_GetRxMessage(&hcan1, CAN_FilterFIFO0, &pRxMsg, buf);
}



