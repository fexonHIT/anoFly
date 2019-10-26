#include "dbus.h"
dbus_t rc;
u16 Rc_dbus_In[6];
uint8_t buf[25]={0};  /*一次接收25个字节*/
void debusInit(void){
   {
   /*初始化外设时钟*/
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
   /*初始化GPIO的时钟*/
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
   }
   /*初始化GPIO*/
   GPIO_InitTypeDef gpio;
   gpio.GPIO_Mode=GPIO_Mode_AF;
   gpio.GPIO_OType=GPIO_OType_PP;
   gpio.GPIO_Pin=GPIO_Pin_6;
   gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;
   gpio.GPIO_Speed=GPIO_Speed_50MHz;
   GPIO_Init(GPIOD,&gpio);
   /*复用引脚映射*/
   GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
   /*初始化USART2*/
   USART_InitTypeDef uart;
   uart.USART_BaudRate=100000;             //DBUS通信的波特率默认是100KHz
   uart.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
   uart.USART_Mode=USART_Mode_Rx;          //只需要处于接收状态即可
   uart.USART_Parity=USART_Parity_Even;    //需要进行偶校验
   uart.USART_StopBits=USART_StopBits_1;
   uart.USART_WordLength=USART_WordLength_8b;
   USART_Init(USART2,&uart);
   USART_Cmd(USART2,ENABLE);
  // uart2DMA1Stream5Init();
   NVIC_InitTypeDef  nvic;
   nvic.NVIC_IRQChannel=USART2_IRQn;
   nvic.NVIC_IRQChannelCmd=ENABLE;
   nvic.NVIC_IRQChannelPreemptionPriority=0;
   nvic.NVIC_IRQChannelSubPriority=0;
   NVIC_Init(&nvic);
   /*开启串口空闲中断*/
   USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
    USART_Cmd(USART2,ENABLE);
}
void uart2DMA1Stream5Init(void){
     {
     /*使能DMA时钟*/
    //   RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
     }
     /*复位DMA*/
     DMA_DeInit(DMA1_Stream5);
     while(DMA_GetCmdStatus(DMA1_Stream5)!=DISABLE){
           ;
     }
     /*初始化DMA*/
     DMA_InitTypeDef dma;
     dma.DMA_BufferSize=25;
     dma.DMA_Channel=DMA_Channel_4;
     dma.DMA_DIR=DMA_DIR_PeripheralToMemory;
     dma.DMA_FIFOMode=DMA_FIFOMode_Disable;   /*关闭DMA的FIFO模式*/
     dma.DMA_FIFOThreshold=DMA_FIFOThreshold_Full;
     dma.DMA_Memory0BaseAddr=(uint32_t)buf;
     dma.DMA_MemoryBurst=DMA_MemoryBurst_Single;
     dma.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
     dma.DMA_MemoryInc=DMA_MemoryInc_Enable;
     dma.DMA_PeripheralBaseAddr=(uint32_t)&(USART2->DR);
     dma.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;
     dma.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
     dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
     dma.DMA_Priority=DMA_Priority_VeryHigh;   /*将DMA的优先级设置为最高*/
     dma.DMA_Mode=DMA_Mode_Normal;
     DMA_Init(DMA1_Stream5,&dma);
         /*开启DMA的传输完成中断*/
     NVIC_InitTypeDef  nvic;
     nvic.NVIC_IRQChannel=DMA1_Stream5_IRQn;
     nvic.NVIC_IRQChannelCmd=ENABLE;
     nvic.NVIC_IRQChannelPreemptionPriority=0;
     nvic.NVIC_IRQChannelSubPriority=0;
     NVIC_Init(&nvic);
     DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
       /*配置串口中断的优先级,初始化NVIC通道*/
     DMA_Cmd(DMA1_Stream5,ENABLE);
//        /*开启一次DMA传输*/
//      USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
            /*等到DMA成功开启*/
      while(DMA_GetCmdStatus(DMA1_Stream5)==DISABLE){
           ;
     }   /*等到DMA成功开启*/


}
/*串口2的空闲中断*/
void  USART2_IRQHandler(void){
      if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET){
           /*关闭空闲中断*/
         USART_ITConfig(USART2,USART_IT_IDLE,DISABLE);
         /*开启一次DMA传输*/
         USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
         /*清除中断标志位*/
        (void)USART2->DR;
        (void)USART2->SR;
      }
      if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
         USART_ClearITPendingBit(USART2,USART_IT_RXNE);
         static uint8_t cnt=0;
         buf[cnt]=USART2->DR;
         if(buf[0]==0x0f){
             cnt++;
         }
         else cnt=0;
         if(cnt==25){
             cnt=0;
             remoteDataProcess(buf);
         }
      }
}
/*DMA1传输完成中断*/
void  DMA1_Stream5_IRQHandler(void){
          if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_TCIF5)==ENABLE){
             DMA_ClearITPendingBit(DMA1_Stream5,DMA_IT_TCIF5);
             USART_DMACmd(USART2,USART_DMAReq_Rx,DISABLE);
             remoteDataProcess(buf);
             // USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
             USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
          }

}
/*对接收到的内容进行处理*/
void  remoteDataProcess(uint8_t *buf){
      if(buf[0]==0x0F){
	      /*buf[0]全8作为低8,buf[1]低三位作为高三位*/
            rc.ch0_rol = (uint16_t)buf[1] | (uint16_t)(buf[2] & 0x07) << 8;
	       /*buf[1]高5位作为低5位，buf[2]低6位作为高6位*/
            rc.ch1_pit = (((uint16_t)(buf[2] & 0xF8)) >> 3) |(((uint16_t)(buf[3] & 0x3F)) << 5);
	        /*buf[2]高2位作为低2位，buf[3]全8位作为中8位，buf[4]最低1位作为最高1位*/
            rc.ch2_thr = ((uint16_t)(buf[3] & 0xC0)) >> 6 | ((uint16_t)(buf[4])) << 2 |((uint16_t)(buf[5] & 0x01)) << 10;
            /*buf[4]高7位作为低7位，buf[5]低四位作为高四位*/
            rc.ch3_yaw = ((uint16_t)buf[5] & 0xFE) >> 1 | (((uint16_t)(buf[6] & 0x0F)) << 7);
             switch(buf[8]){
                 case 0x02:rc.s1=0;break;
                 case 0x00:rc.s1=1;break;
                 case 0xFF:rc.s1=2;break;
             }
              switch(buf[10]){
                 case 0x18:rc.s2=0;break;
                 case 0x10:rc.s2=1;break;
                 case 0x07:rc.s2=2;break;
             }
             Rc_dbus_In[0]=rc.ch0_rol;Rc_dbus_In[1]=rc.ch1_pit;
             Rc_dbus_In[2]=rc.ch2_thr;Rc_dbus_In[3]=rc.ch3_yaw;
      }
      else return ;
      if(buf[23]==0x0c) {
      /*遥控器没有打开*/
          flag.rc_loss=1;     /*表示飞机失去遥控器的控制*/
      }
      else{
          flag.rc_loss=0;      /*飞机获取遥控器的控制*/
      }

}
