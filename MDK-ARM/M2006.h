#ifndef _RM3508_H_
#define _RM3508_H_

#include "can.h"

void M2006_CAN_Send(short * data);
short M2006_Feedback(unsigned char * data);

#endif
