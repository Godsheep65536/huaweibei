#include "PID.h"

PID M2006_PID = {6,0,15,10000,0,0,0,0};
PID M2006_PID_Speed = {10,0,1,10000,0,0,0,0};
/**
 * @brief ��׼λ��ʽPID
 * @param[in] current ʵ��ֵ
 * @param[in] expected ����ֵ
 * @param[in] parameter PID����
 */
void PID_Control(float current, float expected, PID *parameter) {
    parameter->error_last = parameter->error_now;
    parameter->error_now = expected - current;
    parameter->error_inter += parameter->error_now;

    if (parameter->error_inter > parameter->limit)
        parameter->error_inter = parameter->limit;
    if (parameter->error_inter < -parameter->limit)
        parameter->error_inter = -parameter->limit;
    parameter->pid_out = parameter->Kp * parameter->error_now + parameter->Ki * parameter->error_inter +
                         parameter->Kd * (parameter->error_now - parameter->error_last);
}

/**
 * @brief ����ʽPID
 * @param[in] current ʵ��ֵ
 * @param[in] expect ����ֵ
 * @param[in] parameter PID����
 * @return PID����
 */
float PID_Increment(float current, float expect, PID_ADD *parameter) {
    parameter->error_now = expect - current;

    parameter->increament =
            parameter->Kp * (parameter->error_now - parameter->error_next) + parameter->Ki * (parameter->error_now) +
            parameter->Kd * (parameter->error_now - 2 * parameter->error_next + parameter->error_last);

    parameter->error_last = parameter->error_next;
    parameter->error_next = parameter->error_now;

    return parameter->increament;
}

/**
 * @brief ��ʷ��˹Ԥ������λ��ʽPID
 * @param[in] current ʵ��ֵ
 * @param[in] expected ����ֵ
 * @param[in] parameter PID����
 * @param[in] speed ʵ���ٶ�
 */
void PID_Control_Smis(float current, float expected, PID_Smis *parameter, float speed) {
    parameter->error_now = expected - current;
    parameter->error_inter += parameter->error_now;

    if (parameter->error_inter > parameter->limit)
        parameter->error_inter = parameter->limit;
    if (parameter->error_inter < -parameter->limit)
        parameter->error_inter = -parameter->limit;

    parameter->pid_out = parameter->Kp * parameter->error_now + parameter->Ki * parameter->error_inter +
                         parameter->Kd * speed;
}