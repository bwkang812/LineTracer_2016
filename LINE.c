#include<stm32f10x.h>
#include<stdio.h>
#include"delay.h"
#include"LCD.h"

//------------------------------------------------------------------------------
#define redius 25    //�̸� ����.
#define Pi     3.141592 
#define step 400
#define clock (u32)18000000
#define S (u32)(2*Pi*redius/step*clock)
#define count(V) (u16)(S/(u32)V)

//------------------------------------------------------------------------------
/* USART_1 & printf ���� define*/
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ascii)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ascii, FILE *f)
#endif 
#define BUFFER_SIZE 200


//���Ͱ��ú��� -----------------------------------------------------------------


u16 mot_phs_R[8]={0x0900,0x0100,0x0500,0x0400,0x0600,0x0200,0x0A00,0x0800};
int Phase_R;

u16 mot_phs_L[8]={0x9000,0x1000,0x5000,0x4000,0x6000,0x2000,0xA000,0x8000};
int Phase_L;

// BlueTooth ���� ������ ���� ����----------------------------------------------
volatile char control_data;
u8 RxBuf[BUFFER_SIZE];                    
u8 TxBuf[BUFFER_SIZE]; 
u8 TxInCnt=0;   
u8 TxOutCnt=0;     
u8 RxCnt=0;
volatile u16 a=0;
//------------------------------------------------------------------------------
//�߱����� ���� 
int num =0;
u16 LED[14]={0x0001,0x0002,0x0004,0x0008,0x0010,0x0020,0x0040,0x0080,0x0100,0x0200,0x0400,0x0800,0x1000,0x2000};
//ADC ���� ����
u8 AD_Value[14];
int Max[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int min[14]={255,255,255,255,255,255,255,255,255,255,255,255,255,255};
//�����߽ɹ� ���� ����
int percent[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int Length[14]={0,0,0,-53,-38,-23,-8,8,23,38,53,0,0,0};
int error=0;

//PID ���� ����
u16 set_speed=310;

u16 R_speed =310;
u16 L_speed =310;
int KP=16;
int Turn=0;

float last_error=0;
float derivative=0;
float D_data=0;
float KD=0.1;

u16 Lim_min=300;
u16 Lim_max=2000;

//�޴� ���� 
int Push_count=0;


//�ý��� �ֱ� ����
int System_Count=0;
int Div_5_Count=0;

//���� �˰��� ����----------

//���۰���.
int speed_count = 0;
u16 determined_speed=800;

//��������
int End_count = 0;
u16 Decreasing_speed=8;


//ũ�ν� ����ó��. 
int Cross_Delay = 0;			    			
int Cross_count =0;
int Cross_Delay_time=100;

//������ ����ó�� 
int Start_Delay = 1;				
int Start_count = 0;

//----------------------------

//------------------------------------------------------------------------------
void Rcc_Initialize(void) ;
void Gpio_Initialize(void);
void Nvic_Initialize(void);
void EXTI_Initialize(void);
void TIM2_Initialize(void);
void TIM2_IRQHandler(void);
void UART1_Initialize(void);
void USART1_IRQHandler(void);
void TIM4_Initialize(void);// ���� R
void TIM3_Initialize(void);// ���� L
void TIM3_IRQHandler(void);
void ADC1_Initialize(void);

void Get_Maxmin(void);
void Centroid (void);
void Get_ADC(void);//ADC�� �޴� �Լ�
void PID(void);
void Buser_ON(void);
 



void main(void)
{
  Rcc_Initialize( );
  Gpio_Initialize( );
  Lcd_Initialize( );
  Nvic_Initialize( );
  EXTI_Initialize( );
  TIM2_Initialize( ); //Ÿ�̸�2 ����
  UART1_Initialize( );
  USART1_IRQHandler( );
  TIM4_Initialize( );// ���� R
  TIM3_Initialize( );// ����  L
  ADC1_Initialize( );
  
  
  Delay_ms(100);
  
  Lcd_Data_String(0x80,"Hello   " );  // LCD ��� 
  Lcd_Data_String(0xc0," Maestro" );
  //Lcd_Data_String(0x80,"B W KANG" );  // LCD ��� 
 // Lcd_Data_String(0xc0,"20160127" );
  
  
 
 while(1)
 { 
   
   printf("A");
   
   switch(Push_count)
   {
   case 1:
     
    Lcd_Data_String(0x80,"1.      " );  // LCD ��� 
    Lcd_Data_String(0xc0,"Sensing " );
  
    while(Push_count==1)
    {
     Get_ADC( );
     
    }
     break;
     
   case 2:
  Lcd_Data_String(0x80,"2. Speed" );  // LCD ��� 
  
  while(Push_count==2)
  {
    if(determined_speed==800)
    {
      Lcd_Data_String(0xc0,"speed=08" );
      KP=16;
    }
    if(determined_speed==1000)
    {
      Lcd_Data_String(0xc0,"speed=10" );
      KP=20;
      Decreasing_speed=10;
    }
     if(determined_speed==1100)
    {
      Lcd_Data_String(0xc0,"speed=11" );
      KP=22;
      Lim_max=1600;
      KD=0.1;
      Cross_Delay_time=50;
      Decreasing_speed=15;
    }
    if(determined_speed==1200)
    {
      Lcd_Data_String(0xc0,"speed=12" );
      KP=24;
      Lim_min=580;
      Lim_max=2100;
      KD=0.01;
      Cross_Delay_time=50;
      Decreasing_speed=18;
    }
        if(determined_speed==1300)
    {
      Lcd_Data_String(0xc0,"speed=13" );
      KP=25;
      Lim_min=595;
      Lim_max=2275;
      KD=0.02;
      Cross_Delay_time=50;
      Decreasing_speed=20;
    }
    if(determined_speed==1400)Lcd_Data_String(0xc0,"speed=14" );
    if(determined_speed==1600)Lcd_Data_String(0xc0,"speed=16" );
    if(determined_speed==1800)Lcd_Data_String(0xc0,"speed=18" );
    if(determined_speed==2000)Lcd_Data_String(0xc0,"speed=20" );
  }
     break;
  
   case 3:
  Lcd_Data_String(0x80,"3. KP   " );  // LCD ���
  
   while(Push_count==3)
  {
  if(KP==16)Lcd_Data_String(0xc0,"KP=   16" );
  if(KP==17)Lcd_Data_String(0xc0,"KP=   17" );
  if(KP==18)Lcd_Data_String(0xc0,"KP=   18" );
  if(KP==19)Lcd_Data_String(0xc0,"KP=   19" );
  if(KP==20)Lcd_Data_String(0xc0,"KP=   20" );
  if(KP==21)Lcd_Data_String(0xc0,"KP=   21" );
  if(KP==22)Lcd_Data_String(0xc0,"KP=   22" );
  if(KP==23)Lcd_Data_String(0xc0,"KP=   23" );
  if(KP==24)Lcd_Data_String(0xc0,"KP=   24" );
  if(KP==25)Lcd_Data_String(0xc0,"KP=   25" );
  if(KP==26)Lcd_Data_String(0xc0,"KP=   26" );
  if(KP==27)Lcd_Data_String(0xc0,"KP=   27" );
  if(KP==28)Lcd_Data_String(0xc0,"KP=   28" );
  if(KP==29)Lcd_Data_String(0xc0,"KP=   29" );
  if(KP==30)Lcd_Data_String(0xc0,"KP=   30" );
  if(KP==31)Lcd_Data_String(0xc0,"KP=   31" );
  if(KP==32)Lcd_Data_String(0xc0,"KP=   32" );
  if(KP==33)Lcd_Data_String(0xc0,"KP=   33" );
  if(KP==34)Lcd_Data_String(0xc0,"KP=   34" );
  if(KP==35)Lcd_Data_String(0xc0,"KP=   35" );
  if(KP==36)Lcd_Data_String(0xc0,"KP=   36" );
  if(KP==37)Lcd_Data_String(0xc0,"KP=   37" );
  if(KP==38)Lcd_Data_String(0xc0,"KP=   38" );
  if(KP==39)Lcd_Data_String(0xc0,"KP=   39" );
  if(KP==40)Lcd_Data_String(0xc0,"KP=   40" );
  }
  
  
     break;
   case 4:
     
     Delay_ms(1000);
     
  Lcd_Data_String(0x80,"3.      " );  // LCD ��� 
  Lcd_Data_String(0xc0,"Start   " );
  
  

  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 
  TIM_Cmd(TIM4, ENABLE); 
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
  TIM_Cmd(TIM3, ENABLE);
  
  Delay_ms(500);
  
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //�ý��� �ֱ�.
  TIM_Cmd(TIM2, ENABLE);
  
     
       

  
   while(Push_count==4)
    {
      
      //�����Լ�.
	if (percent[0]<10&&percent[1]<10&&percent[2]<10&&percent[3]<10 && percent[4]<10 && percent[5]<10 && percent[6]<10 && percent[7]<10 && percent[8]<10&& percent[9]<10&&percent[10]<10&&percent[11]<10&&percent[12]<10&&percent[13]<10 )
	{				
          TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);  //������ 				
          TIM_Cmd(TIM3, DISABLE);				
          TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);  //���� 				
          TIM_Cmd(TIM4, DISABLE);	
          TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE); 
          TIM_Cmd(TIM2, DISABLE);
          
          GPIOC->ODR = 0x0080;   //Buser On
          Delay_ms(100);
          GPIOC->ODR = 0x0000;   //Buser Off
          Delay_ms(100);
          GPIOC->ODR = 0x0080;   //Buser On
          Delay_ms(100);
          GPIOC->ODR = 0x0000;   //Buser Off
          Push_count=5;
          
          
        }
      
      
       if(System_Count==1)
       {
         
        //���۰���-------- ���۹�ư�� �����ڸ��� Ż������ ���� �����Ѵ�. 
        if (speed_count == 0)
        {
          set_speed = set_speed + 10  ;
        
          if (set_speed>determined_speed)
	      speed_count=1;
        
        }
        //----------------
        
        //������ ����ó�� 
        
        if (Start_Delay == 1)			
        {	
          Start_count=Start_count+5;	//5ms���� 5�� ������Ų��. 			
        }			
        if (Start_count == 1000)	        //1000ms�� ������ ����. 		
        {				
          Start_Delay = 0;				
          Start_count = 0;	
         
        }
        
        //----------------
        
        //cross ����ó��---
        if ((percent[4]>80 && percent[5]>80 && percent[6]>80&&percent[7]>80&& percent[8]>80) || (percent[5]>80&&percent[6]>80&&percent[7]>80 && percent[8]>80 && percent[9]>80))			
        {	
          GPIOC->ODR = 0x0080;   //Buser On
          error = 0;					
          Cross_Delay = 1;			
        }
             		
        if (Cross_Delay == 1)			
        {			
          Cross_count=Cross_count+5;	//5ms���� 5�� ������Ų��. 			
        }			
        if (Cross_count == Cross_Delay_time)	        //Cross_Delay_time=100ms�� ������ ����. 		
        {				
          Cross_Delay = 0;				
          Cross_count = 0;	
          GPIOC->ODR = 0x0000;   //Buser Off
        }
        
        //-----------------
        
        //����(����)------���帶ũ���� �����Ѵ�.
        if (percent[1]>60 && percent[12]>60 && (percent[6]>50 || percent[7]>50) && (percent[4]<50 || percent[5]<50) &&(percent[8]<50 || percent[9]<50)&&Cross_Delay == 0&&Start_Delay == 0)		
        {
            GPIOC -> ODR =0x0080;               //Buser on
            
            End_count = 1;
	}			
        
        if (End_count == 1)                         
             set_speed = set_speed - Decreasing_speed;   //down_speed;	
	
        if (set_speed<100)			
        {		
           GPIOC -> ODR =0x0000;               //Buser off
          
           TIM_ITConfig(TIM3, TIM_IT_Update,DISABLE); 
           TIM_Cmd(TIM3, DISABLE);
           TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE); 
           TIM_Cmd(TIM4, DISABLE);
           TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE); 
           TIM_Cmd(TIM2, DISABLE);
           Push_count=5;
           
        }
        //----------------
        
         Get_ADC( );
         PID( );
         
         System_Count=0;
       }
       
    }
     break;
   
   case 5:
     
  Lcd_Data_String(0x80,"4.      " );  // LCD ��� 
  Lcd_Data_String(0xc0,"Stop    " );
  
  TIM_ITConfig(TIM3, TIM_IT_Update,DISABLE); 
  TIM_Cmd(TIM3, DISABLE);
  TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE); 
  TIM_Cmd(TIM4, DISABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE); 
  TIM_Cmd(TIM2, DISABLE);
     break;
     
     
   }
   
  //printf("%d %d\n",L_speed,R_speed);
  //GPIOC -> ODR =0x0080;             //Buser On
   
 }
  
}

//------------------------------------------------------------------------------
void Buser_ON(void)
 {
  GPIOC -> ODR =0x0080;               //Buser on
 }
//------------------------------------------------------------------------------
//ADC�� �޴� �Լ� 
void Get_ADC(void)
    {      
      for(num=0;num<14;num++)
      {
       GPIOD->ODR = LED[num];  //�߱� on  
        
      ADC_RegularChannelConfig(ADC1, num,  1, ADC_SampleTime_13Cycles5);
      ADC_Cmd(ADC1,ENABLE);
      while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET); //��ȯ�� ������ End of Conversion Stais�� Ȱ��ȭ �Ǹ� �����͸� �������
      AD_Value[num]=(ADC1->DR)>>4; 
        
      GPIOD->ODR = 0x0000;  //�߱� off
      
      Get_Maxmin( );
      Centroid ( );
     
      //Delay_us(500);
      //printf("%d:%d\n",num,AD_Value[num]);
      
      }
    }

//Get_Maxmin--------------------------------------------------------------------

void Get_Maxmin(void)
{
 Max[num]=(Max[num]>AD_Value[num])?Max[num]:AD_Value[num];  
  
 min[num]=(min[num]<AD_Value[num])?min[num]:AD_Value[num];
  
 //Delay_us(500);
 //printf("%d: %d %d\n",num,Max[num],min[num]);
}

//�����߽ɹ�--------------------------------------------------------------------

void Centroid (void)
{
 percent[num]=(AD_Value[num]-min[num])*100/(Max[num]-min[num]);
 //Delay_us(500);
 //printf("%d: %d\n",num,percent[num]);
 
 error=(percent[4]*Length[4]+percent[5]*Length[5]+percent[6]*Length[6]+percent[7]*Length[7]+percent[8]*Length[8]+percent[9]*Length[9]) 
              /
             (percent[4]+percent[5]+percent[6]+percent[7]+percent[8]+percent[9]);
 
 //Delay_ms(1);
 //printf("%d\n",error);
}

//------------------------------------------------------------------------------
//PID���� �Լ�

  void PID(void) 
         
         {
           derivative=(error-last_error)*100;
           D_data=KD*derivative;
           last_error=error;
           
           
          Turn=KP*(int)error+(int)D_data;
          L_speed=(u16)(set_speed+Turn);
          R_speed=(u16)(set_speed-Turn);
          
          
          if(L_speed<Lim_min)L_speed=Lim_min;
          if(R_speed<Lim_min)R_speed=Lim_min;
          
          
          if(L_speed>Lim_max)L_speed=Lim_max;
          if(R_speed>Lim_max)R_speed=Lim_max;
          
         }

//------------------------------------------------------------------------------
//RCC ����----------------------------------------------------------------------
void Rcc_Initialize(void)
{
  ErrorStatus HSEStartUpStatus;
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if (HSEStartUpStatus == SUCCESS)
  {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);
    RCC_HCLKConfig(RCC_SYSCLK_Div1);//HCLK = SYSCLK
    
    RCC_PCLK2Config(RCC_HCLK_Div1); // 72Mhz
    RCC_PCLK1Config(RCC_HCLK_Div4); // 72/4 x2 = 36Mhz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); //PLL ����8Mhz*9=72Mhz
    RCC_PLLCmd(ENABLE);
    
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != 0x08);
  }
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOC,ENABLE);//GPIO c�� e�� clock����� �����Ѵ�.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);// Ÿ�̸�2 Ŭ�����
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD,ENABLE); //GPIO A�� D�� clock����Ѵ�. 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//������� UART1 clock���.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);// Ÿ�̸�4 Ŭ�����
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);// Ÿ�̸�3 Ŭ�����
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);// ADC1Ŭ�����
 
}



//GPIO ����---------------------------------------------------------------------
void Gpio_Initialize(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   
   
   //LCD Rs E
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;//�� 8�� 9�� �߰�
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   
    //LCD
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;// �� 1~7�߰�
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOE, &GPIO_InitStructure);
   
     //����.
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //�� 7
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   
     //��ư
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;//�� 10,11,12�� �߰� 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   
     //������� TX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//�� 9�߰� 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   //������� RX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//�� 10�߰� 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //���� TIM4(R)
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; //�� 8,9,10,11
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOE, &GPIO_InitStructure);
   
   
     //���� TIM3(L)
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; //�� 12,13,14,15
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   
   //�߱� 
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;// �� 1~14�߰�
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
   
     //ADC1  A��Ʈ 
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;// �� 0~7   
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analog Input mode. 
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //ADC1  B��Ʈ  
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;// �� 0,1   
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analog Input mode. 
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   
    //ADC1  C��Ʈ  
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;// �� 0,1   
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analog Input mode. 
   GPIO_Init(GPIOC, &GPIO_InitStructure);
} 

// Nvic �Լ�����----------------------------------------------------------
void Nvic_Initialize(void)
{ 
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
   
   //USART1_IRQ enable
   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
  // EXTI15_10_IRQ enable
   NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
   //TIM2_IRQn enable
   NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //enable the TIM2 Interrupt
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
    
   // Tim4_IRQ enable
   NVIC_InitStructure.NVIC_IRQChannel =  TIM4_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
   
   // Tim3_IRQ enable
   NVIC_InitStructure.NVIC_IRQChannel =  TIM3_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
   

   
}


//------------------------------------------------------------------------------
//10.11.12.13.14.15pin �� ����...�ܺ���Ʈ��Ʈ�� ����Ϸ���
//EXTI15_10_IRQChannel ������ �ϰ�



void EXTI15_10_IRQHandler(void) //10.11.12.
{
   if(EXTI_GetITStatus(EXTI_Line10) != RESET) {
 
     
     
     GPIOC -> ODR =0x0080;   //Buser on
    
     KP++;
     
     Delay_ms(100);
     GPIOC -> ODR =0x0000;   //Buser off
   
     Delay_ms(500);
     
     
        
    EXTI_ClearITPendingBit(EXTI_Line10);
        
   }
   if(EXTI_GetITStatus(EXTI_Line11) != RESET) {

   
     GPIOC -> ODR =0x0080;   //Buser on
     
     if(determined_speed>=1000)determined_speed=determined_speed+100;
     
     if(determined_speed<1000)determined_speed=determined_speed+200;
    
    
     Delay_ms(100);
     GPIOC -> ODR =0x0000;   //Buser off
   
     Delay_ms(500);

      EXTI_ClearITPendingBit(EXTI_Line11);
   
   }
   if(EXTI_GetITStatus(EXTI_Line12) != RESET) {

     
     GPIOC -> ODR =0x0080;   //Buser on
    
     Push_count++;
     
     Delay_ms(100);
     GPIOC -> ODR =0x0000;   //Buser off
   
     Delay_ms(500);
     
  
      EXTI_ClearITPendingBit(EXTI_Line12);
   }

   
} 

//---------------------------------------------------------------------
//�ܺ����ͷ�Ʈ ����. 
void EXTI_Initialize(void)
{
   EXTI_InitTypeDef EXTI_InitStructure;
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
   //GPIO C 10,11,12 

   GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);
   EXTI_InitStructure.EXTI_Line = EXTI_Line10;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource11);
   EXTI_InitStructure.EXTI_Line = EXTI_Line11;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);
   
   GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource12);
   EXTI_InitStructure.EXTI_Line = EXTI_Line12;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);
   
}

//--------------------------------------------------------------------------------
//�ý��� �ֱ� 5ms.
void TIM2_IRQHandler(void) //per1ms
{
 
   TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
 
   Div_5_Count++;
   
   if(Div_5_Count%5==0)
   {
   System_Count=1;
   }
}

//------------------------------------------------------------------------------
//�ý��� �ֱ� 5ms. 
void TIM2_Initialize(void)

{   
   
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

   TIM_TimeBaseStructure.TIM_Period = 120; //1~65535  
   TIM_TimeBaseStructure.TIM_Prescaler = 300;     
   //�ð� ��� �� = 36MHZ ���޹޴´�.36000,000  x 1/period  x 1/prescaler  [1200, 300=   10ms]
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  
   TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE); //TIM IT enable
   TIM_Cmd(TIM2, DISABLE); //TIM2 enable counter 

}

//Printf�� ����ϱ� ���� ����---------------------------------------------------
PUTCHAR_PROTOTYPE
{
    TxBuf[TxInCnt] = ascii;
    if(TxInCnt<BUFFER_SIZE-1) TxInCnt++;
    else TxInCnt = 0;     
    
    //Enable the USART1 Transmit interrupt     
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

    return ascii; 
}
  
//-----------------------------------------------------------------------------

void UART1_Initialize(void){ //PCLK2�� ����Ѵ�. 
  
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   
   
   USART_InitStructure.USART_BaudRate = 115200; 
   // baud rate= ������ ��ſ��� ���� ������ ���� �ӵ��� 1�ʰ��� ���۵Ǵ� ��ȣ�� ���� ��Ÿ�� ��.
   
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   //8��Ʈ �Ǵ� 9��Ʈ�� ���� �� ���ִ�. 
   
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   //�ϳ��� �ܾ�(word)�� ���� ǥ���ϱ� ���� ���Ŀ� �ΰ��ϴ� 2��Ʈ(1 1.5 2 �� ���� ����)
   
   USART_InitStructure.USART_Parity = USART_Parity_No;
   //1���� ��Ʈ�� ������ ¦������ Ȧ�������� ����

   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   // ���������� ���źҰ� ������ ��� �۽������� ������ �������� �ʵ��� �ϰ� �ٽ� ���Ű��� ���°� �Ǿ��� ���� ������ �����ϴ� ���. 
   
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   //USART_CR1 �������Ϳ��� bit3 TE Transmitter enable �� Bit 2 RE Receiver enable �� �����ϱ� ����. �ش��Ʈ��ġ�� 1�� �ǵ�����. 
   
   USART_Init(USART1, &USART_InitStructure);
   //���������ϰ� 0���� �ʱ�ȭ. 
   
   USART_Cmd(USART1, DISABLE); // ����Ϸ��� ENABLE ����ߵ� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
   //USART1 Enable
   
   
   NVIC_InitStructure.NVIC_IRQChannel = 37;  //USART1_IRQChannel
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
}


//(BLUETOOTH ���)USART1 ���ͷ�Ʈ ����------------------------------------------
void USART1_IRQHandler(void)
//UART1���� ���ŵ� ���� �д� ��. 
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)// ��Ʈ�� 0�� �ƴ� ��쿡 �����Ͱ� ����.
    {         
       
        control_data  = USART_ReceiveData(USART1); //0�� �ƴ� ��쿡 �����͸� �д´�. control_data= ���ۿ� ���� �����ϴ� ������ �ϴ� ����.
        
        /*Buffer�� �����ؾ��ϴ� ����.
        
        ���Žð������� ó���ð��� �� ���ͷ�Ʈ �߻��κп����� ���ۿ� �ܼ��� ���常�ϰ� ó���� �ٸ������� �Ѵ�.
        */
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
        // pending bit�� clear.(�����ָ� ���ͷ�Ʈ�� ó���Ǿ����� �˼����� �ٽ� ���ͷ�Ʈ�� �߻��Ѱ����� �����ؼ� ��� �ڵ鷯 ȣ��) 
       
    }
    
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)//��Ʈ�� 0�� �ƴ� ��쿡 �����Ͱ� �۽� 
    {         
     
        USART_SendData(USART1,TxBuf[TxOutCnt]);
        
        if(TxOutCnt<BUFFER_SIZE-1) TxOutCnt++; // Txoutcount�� buffer size ���� ������ +1�� ����. 
        else TxOutCnt = 0;      
            
        if(TxOutCnt == TxInCnt)// Txoutcount�� Txincount�� �Ǹ� tx ���� RX Ų��. 
        {
          USART_ITConfig(USART1, USART_IT_TXE, DISABLE); 
          USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        }         
    }        
   
}

void TIM4_Initialize(void)//����  R

{

   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

   TIM_TimeBaseStructure.TIM_Period = count(R_speed);

   TIM_TimeBaseStructure.TIM_Prescaler = 1;  // 36MHz/1+1= 36

   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   
   TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
   
   TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE); 

   TIM_Cmd(TIM4, DISABLE); 
   
  
} 

//-----------------------------------------------------------------------------
void TIM4_IRQHandler(void)//���� R

{

  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  
  TIM4 ->ARR =count(R_speed);  
  
// GPIOE->ODR=(GPIOE->ODR&0xf0ff)|mot_phs_R[Phase_R];
 GPIOE->ODR= mot_phs_R[Phase_R];


  Phase_R++;
  

  if(Phase_R==8)Phase_R=0;
  
  

} 

//------------------------------------------------------------------------------

void TIM3_Initialize(void)//���� L

{

   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

   TIM_TimeBaseStructure.TIM_Period = count(L_speed);

   TIM_TimeBaseStructure.TIM_Prescaler = 1;  // 36MHz/1+1= 18

   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   
   TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
   
   TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE); 

   TIM_Cmd(TIM3, DISABLE); 
   
  
} 


//------------------------------------------------------------------------------
void TIM3_IRQHandler(void)//���� L

{

  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

  
  TIM3 ->ARR =count(L_speed); 

  
 //GPIOB->ODR=(GPIOB->ODR&0x0fff)|mot_phs_L[Phase_L];
 GPIOB->ODR=mot_phs_L[Phase_L];

  Phase_L++;
  

  if(Phase_L==8)Phase_L=0;
  
   

} 

//------------------------------------------------------------------------------
//ADC 

void ADC1_Initialize(void)
{
   ADC_InitTypeDef ADC_InitStructure;

   ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 
   //ADC�� �������� ���. single mode 
   ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
   //��Ʈ�� 0���� ����� DISABLE, ��Ʈ�� 1�� ����� ENABLE
   //ENABLE�ϰ� �Ǹ� ä�ε��� ��ĵ�ϰԵȴ�. ����ä�ο��� conversion�� �����Ѵ�. 
   ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
   //0�����ϸ� ADC��ȯ�� �ѹ��� ����, 1�� �����ϸ� �ݺ��ؼ� ����.
   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
   //regularä�ο� ���� conversion�� �����ϵ����ϴ� trigger�� ���� �� �� �ִ� ��. 
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
   //16��Ʈ�� ����Ÿ �������Ϳ� ��� ������ ���ΰ�?  
   ADC_InitStructure.ADC_NbrOfChannel = 14; 
   //��ȯ�� ä�μ� 14��. 
   ADC_Init(ADC1, &ADC_InitStructure);
   //ADC_TempSensorVrefintCmd(ENABLE); //�µ�����. 
   /*
   ADC_RegularChannelConfig(ADC1, ADC_Channel_0,  1, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_1,  2, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_13Cycles5);
   // ������ ���ø��ϴ� ����Ŭ�� ��. convertion �ð�= sampling time +
   */
   
   ADC_Cmd(ADC1, ENABLE); //Enable   ADC1 
   
   
   ADC_ResetCalibration(ADC1);//���� ������ ���� ���ִ�. 
   while(ADC_GetResetCalibrationStatus(ADC1));//0���� �ٲ𶧱��� ��ٸ�. 

   ADC_StartCalibration(ADC1);
   while(ADC_GetCalibrationStatus(ADC1)); 
  
   
}