/**
 * @file    PID.h
 * @author  yao
 * @date    1-May-2020
 * @brief   PIDģ��ͷ�ļ�
 */

#ifndef _PID_H_
#define _PID_H_

/**
 * @brief �޷��꺯��
 * @param IN �޷�����
 * @param MAX ���ֵ
 * @param MIN ��Сֵ
 */
#define limit(IN, MAX, MIN) \
    if (IN < MIN)           \
        IN = MIN;           \
    if (IN > MAX)           \
        IN = MAX

/**
 * @brief ��׼λ��ʽPID����
 */
typedef struct __PID {
    float Kp;           //!<@brief ����ϵ��
    float Ki;           //!<@brief ����ϵ��
    float Kd;           //!<@brief ΢��ϵ��
    float limit;        //!<@brief �����޷�
    float error_now;    //!<@brief ��ǰ���
    float error_last;   //!<@brief ��һ�����
    float error_inter;  //!<@brief ������
    float pid_out;      //!<@brief PID���
} PID;

/**
 * @brief ��ʷ��˹Ԥ������λ��ʽPID����
 */
typedef struct __PID_Smis {
    float Kp;           //!<@brief ����ϵ��
    float Ki;           //!<@brief ����ϵ��
    float Kd;           //!<@brief ΢��ϵ��
    float limit;        //!<@brief �����޷�
    float error_now;    //!<@brief ��ǰ���
    float error_inter;  //!<@brief ������
    float pid_out;      //!<@brief PID���
} PID_Smis;

/**
 * @brief ����ʽPID����
 */
typedef struct __PID_ADD {
    float Kp;           //!<@brief ����ϵ��
    float Ki;           //!<@brief ����ϵ��
    float Kd;           //!<@brief ΢��ϵ��
    float error_now;    //!<@brief ��ǰ���
    float error_next;   //!<@brief ��һ�����
    float error_last;   //!<@brief ���ϴ����
    float increament;   //!<@brief PID����
} PID_ADD;

extern PID M2006_PID;
extern PID M2006_PID_Speed;
extern PID M2006_PID_zero;

void PID_Control(float current, float expected, PID *data);

void PID_Control_Smis(float current, float expected, PID_Smis *data, float speed);

float PID_Increment(float current, float expect, PID_ADD *parameter);

#endif