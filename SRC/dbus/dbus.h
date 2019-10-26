#ifndef __DBUS_H
#define __DBUS_H
#include "include.h"
/*DJ6遥控器通道数据类型定义*/
enum{
    OFF=0,
    CL,
    HL,
};
enum{
    GPS=0,
    ALTI,
    ATTI,
};
typedef __packed struct{
      int16_t ch0_rol;
      int16_t ch1_pit;
      int16_t ch2_thr;
      int16_t ch3_yaw;   
      uint8_t s1;
      uint8_t s2;
}dbus_t;
extern dbus_t rc;
extern u16 Rc_dbus_In[6];
void debusInit(void);
void uart2DMA1Stream5Init(void);
void  remoteDataProcess(uint8_t *buf);
#endif  /*__DBUS_H*/
