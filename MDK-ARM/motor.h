#ifndef _MOTOR_H_
#define _MOTOR_H_

#if defined(STM32F407xx) || defined(STM32F405xx) || defined(STM32F427xx)
#include <stm32f4xx.h>
#elif defined(STM32F303xx) || defined(STM32F334xx)
#include <stm32f3xx.h>
#elif defined(STM32F103xx)
#include <stm32f1xx.h>
#endif

#include "CANDrive.h"

#define RM3508_LIMIT 16384  //!<@brief RM3508的输出限幅
#define GM6020_LIMIT 30000  //!<@brief GM6020的输出限幅
#define RM3510_LIMIT 32767  //!<@brief RM3510的输出限幅
#define GM3510_LIMIT 29000  //!<@brief GM3510的输出限幅
#define M2006_LIMIT  10000  //!<@brief M2006 的输出限幅
#define RM6623_LIMIT 32767  //!<@brief RM6623的输出限幅(找不到了)

/**
 * @brief M2006电机数据结构体
 */
typedef struct __M2006_TypeDef {
    uint16_t MchanicalAngle;    //!<@brief 机械角度
    int16_t Speed;              //!<@brief 转速
    uint16_t LsatAngle;         //!<@brief 上一次的机械角度
    int16_t r;                  //!<@brief 圈数
    int32_t Angle;              //!<@brief 连续化机械角度
    float Angle_DEG;            //!<@brief 连续化角度制角度
} M2006_TypeDef;


extern M2006_TypeDef M2006_dealcards;

void M2006_Receive(M2006_TypeDef *Dst, uint8_t *Data);


HAL_StatusTypeDef MotorSend(can_num_e can, uint32_t STD_ID, int16_t *Data);

#endif
