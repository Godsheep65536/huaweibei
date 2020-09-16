#include <motor.h>
#include <CANDrive.h>

M2006_TypeDef M2006_dealcards;


/**
 * @brief M2006���ݽ���
 * @param[out] Dst M2006������ݽṹ��ָ��
 * @param[in] Data CAN����ָ֡��
 */
void M2006_Receive(M2006_TypeDef *Dst, uint8_t *Data) {
    Dst->MchanicalAngle = (uint16_t)(Data[0] << 8 | Data[1]);
    Dst->Speed = (int16_t)(Data[2] << 8 | Data[3]);

    int16_t diff = Dst->MchanicalAngle - Dst->LsatAngle;

    if (diff > 4000)
        Dst->r--;
    if (diff < -4000)
        Dst->r++;

    Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
    Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
    Dst->LsatAngle = Dst->MchanicalAngle;
}

/**
 * @brief ���͵�������ź�
 * @param[in] can CANö��
 * @param[in] STD_ID ��׼֡ID
 * @param[in] Data ��������ź�����ָ��
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef MotorSend(can_num_e can, uint32_t STD_ID, int16_t *Data) {
    uint8_t temp[8];
    temp[0] = (uint8_t)(Data[0] >> 8);
    temp[1] = (uint8_t)(Data[0] & 0xff);
    temp[2] = (uint8_t)(Data[1] >> 8);
    temp[3] = (uint8_t)(Data[1] & 0xff);
    temp[4] = (uint8_t)(Data[2] >> 8);
    temp[5] = (uint8_t)(Data[2] & 0xff);
    temp[6] = (uint8_t)(Data[3] >> 8);
    temp[7] = (uint8_t)(Data[3] & 0xff);
    if (can == can1)
        return CAN1_Send_Msg(STD_ID, temp);
#ifdef CAN2_SUPPORT
        else if (can == can2)
            return CAN2_Send_Msg(STD_ID, temp);
#endif
    else
        return HAL_ERROR;
}