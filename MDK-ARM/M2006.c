#include "M2006.h"
#include "can.h"
#include "test_can.h"

void M2006_CAN_Send(short * data)//遥控器发送数据
{
	unsigned char buff[8];
	buff[0] = (unsigned char)((data[0]>>8)&0xff);
	buff[1] = (unsigned char)((data[0])&0xff);
	buff[2] = (unsigned char)((data[1]>>8)&0xff);
	buff[3] = (unsigned char)((data[1])&0xff);
	buff[4] = (unsigned char)((data[2]>>8)&0xff);
	buff[5] = (unsigned char)((data[2])&0xff);
	buff[6] = (unsigned char)((data[3]>>8)&0xff);
	buff[7] = (unsigned char)((data[3])&0xff);
	CAN1_Send_Msg(0x200,buff);
}

short M2006_Feedback(unsigned char * data)//反馈
{
	static short buff;
	buff = data[2];
	buff = (buff<<8)|data[2];
	return -buff;
}
