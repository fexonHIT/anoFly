#include "dbus.h"
dbus_t rc;
u16 Rc_dbus_In[6];
uint8_t buf[25]={0};  /*һ�ν���25���ֽ�*/
void debusInit(void){
   {
   /*��ʼ������ʱ��*/
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
   /*��ʼ��GPIO��ʱ��*/
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
   }
   /*��ʼ��GPIO*/
   GPIO_InitTypeDef gpio;
   gpio.GPIO_Mode=GPIO_Mode_AF;
   gpio.GPIO_OType=GPIO_OType_PP;
   gpio.GPIO_Pin=GPIO_Pin_6;
   gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;
   gpio.GPIO_Speed=GPIO_Speed_50MHz;
   GPIO_Init(GPIOD,&gpio);
   /*��������ӳ��*/
   GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
   /*��ʼ��USART2*/
   USART_InitTypeDef uart;
   uart.USART_BaudRate=100000;             //DBUSͨ�ŵĲ�����Ĭ����100KHz
   uart.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
   uart.USART_Mode=USART_Mode_Rx;          //ֻ��Ҫ���ڽ���״̬����
   uart.USART_Parity=USART_Parity_Even;    //��Ҫ����żУ��
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
   /*�������ڿ����ж�*/
   USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
    USART_Cmd(USART2,ENABLE);
}
void uart2DMA1Stream5Init(void){
     {
     /*ʹ��DMAʱ��*/
    //   RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
     }
     /*��λDMA*/
     DMA_DeInit(DMA1_Stream5);
     while(DMA_GetCmdStatus(DMA1_Stream5)!=DISABLE){
           ;
     }
     /*��ʼ��DMA*/
     DMA_InitTypeDef dma;
     dma.DMA_BufferSize=25;
     dma.DMA_Channel=DMA_Channel_4;
     dma.DMA_DIR=DMA_DIR_PeripheralToMemory;
     dma.DMA_FIFOMode=DMA_FIFOMode_Disable;   /*�ر�DMA��FIFOģʽ*/
     dma.DMA_FIFOThreshold=DMA_FIFOThreshold_Full;
     dma.DMA_Memory0BaseAddr=(uint32_t)buf;
     dma.DMA_MemoryBurst=DMA_MemoryBurst_Single;
     dma.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
     dma.DMA_MemoryInc=DMA_MemoryInc_Enable;
     dma.DMA_PeripheralBaseAddr=(uint32_t)&(USART2->DR);
     dma.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;
     dma.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
     dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
     dma.DMA_Priority=DMA_Priority_VeryHigh;   /*��DMA�����ȼ�����Ϊ���*/
     dma.DMA_Mode=DMA_Mode_Normal;
     DMA_Init(DMA1_Stream5,&dma);
         /*����DMA�Ĵ�������ж�*/
     NVIC_InitTypeDef  nvic;
     nvic.NVIC_IRQChannel=DMA1_Stream5_IRQn;
     nvic.NVIC_IRQChannelCmd=ENABLE;
     nvic.NVIC_IRQChannelPreemptionPriority=0;
     nvic.NVIC_IRQChannelSubPriority=0;
     NVIC_Init(&nvic);
     DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
       /*���ô����жϵ����ȼ�,��ʼ��NVICͨ��*/
     DMA_Cmd(DMA1_Stream5,ENABLE);
//        /*����һ��DMA����*/
//      USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
            /*�ȵ�DMA�ɹ�����*/
      while(DMA_GetCmdStatus(DMA1_Stream5)==DISABLE){
           ;
     }   /*�ȵ�DMA�ɹ�����*/


}
/*����2�Ŀ����ж�*/
void  USART2_IRQHandler(void){
      if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET){
           /*�رտ����ж�*/
         USART_ITConfig(USART2,USART_IT_IDLE,DISABLE);
         /*����һ��DMA����*/
         USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
         /*����жϱ�־λ*/
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
/*DMA1��������ж�*/
void  DMA1_Stream5_IRQHandler(void){
          if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_TCIF5)==ENABLE){
             DMA_ClearITPendingBit(DMA1_Stream5,DMA_IT_TCIF5);
             USART_DMACmd(USART2,USART_DMAReq_Rx,DISABLE);
             remoteDataProcess(buf);
             // USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
             USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
          }

}
/*�Խ��յ������ݽ��д���*/
void  remoteDataProcess(uint8_t *buf){
      if(buf[0]==0x0F){
	      /*buf[0]ȫ8��Ϊ��8,buf[1]����λ��Ϊ����λ*/
            rc.ch0_rol = (uint16_t)buf[1] | (uint16_t)(buf[2] & 0x07) << 8;
	       /*buf[1]��5λ��Ϊ��5λ��buf[2]��6λ��Ϊ��6λ*/
            rc.ch1_pit = (((uint16_t)(buf[2] & 0xF8)) >> 3) |(((uint16_t)(buf[3] & 0x3F)) << 5);
	        /*buf[2]��2λ��Ϊ��2λ��buf[3]ȫ8λ��Ϊ��8λ��buf[4]���1λ��Ϊ���1λ*/
            rc.ch2_thr = ((uint16_t)(buf[3] & 0xC0)) >> 6 | ((uint16_t)(buf[4])) << 2 |((uint16_t)(buf[5] & 0x01)) << 10;
            /*buf[4]��7λ��Ϊ��7λ��buf[5]����λ��Ϊ����λ*/
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
      /*ң����û�д�*/
          flag.rc_loss=1;     /*��ʾ�ɻ�ʧȥң�����Ŀ���*/
      }
      else{
          flag.rc_loss=0;      /*�ɻ���ȡң�����Ŀ���*/
      }

}
