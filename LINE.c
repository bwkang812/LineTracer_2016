#include<stm32f10x.h>
#include<stdio.h>
#include"delay.h"
#include"LCD.h"

//------------------------------------------------------------------------------
#define redius 25    //미리 미터.
#define Pi     3.141592 
#define step 400
#define clock (u32)18000000
#define S (u32)(2*Pi*redius/step*clock)
#define count(V) (u16)(S/(u32)V)

//------------------------------------------------------------------------------
/* USART_1 & printf 관련 define*/
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ascii)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ascii, FILE *f)
#endif 
#define BUFFER_SIZE 200


//모터관련변수 -----------------------------------------------------------------


u16 mot_phs_R[8]={0x0900,0x0100,0x0500,0x0400,0x0600,0x0200,0x0A00,0x0800};
int Phase_R;

u16 mot_phs_L[8]={0x9000,0x1000,0x5000,0x4000,0x6000,0x2000,0xA000,0x8000};
int Phase_L;

// BlueTooth 수신 데이터 저장 변수----------------------------------------------
volatile char control_data;
u8 RxBuf[BUFFER_SIZE];                    
u8 TxBuf[BUFFER_SIZE]; 
u8 TxInCnt=0;   
u8 TxOutCnt=0;     
u8 RxCnt=0;
volatile u16 a=0;
//------------------------------------------------------------------------------
//발광관련 변수 
int num =0;
u16 LED[14]={0x0001,0x0002,0x0004,0x0008,0x0010,0x0020,0x0040,0x0080,0x0100,0x0200,0x0400,0x0800,0x1000,0x2000};
//ADC 관련 변수
u8 AD_Value[14];
int Max[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int min[14]={255,255,255,255,255,255,255,255,255,255,255,255,255,255};
//무게중심법 관련 변수
int percent[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int Length[14]={0,0,0,-53,-38,-23,-8,8,23,38,53,0,0,0};
int error=0;

//PID 관련 변수
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

//메뉴 변수 
int Push_count=0;


//시스템 주기 변수
int System_Count=0;
int Div_5_Count=0;

//주행 알고리즘 변수----------

//시작가속.
int speed_count = 0;
u16 determined_speed=800;

//정지감속
int End_count = 0;
u16 Decreasing_speed=8;


//크로스 예외처리. 
int Cross_Delay = 0;			    			
int Cross_count =0;
int Cross_Delay_time=100;

//시작점 예외처리 
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
void TIM4_Initialize(void);// 모터 R
void TIM3_Initialize(void);// 모터 L
void TIM3_IRQHandler(void);
void ADC1_Initialize(void);

void Get_Maxmin(void);
void Centroid (void);
void Get_ADC(void);//ADC값 받는 함수
void PID(void);
void Buser_ON(void);
 



void main(void)
{
  Rcc_Initialize( );
  Gpio_Initialize( );
  Lcd_Initialize( );
  Nvic_Initialize( );
  EXTI_Initialize( );
  TIM2_Initialize( ); //타이머2 부저
  UART1_Initialize( );
  USART1_IRQHandler( );
  TIM4_Initialize( );// 모터 R
  TIM3_Initialize( );// 모터  L
  ADC1_Initialize( );
  
  
  Delay_ms(100);
  
  Lcd_Data_String(0x80,"Hello   " );  // LCD 출력 
  Lcd_Data_String(0xc0," Maestro" );
  //Lcd_Data_String(0x80,"B W KANG" );  // LCD 출력 
 // Lcd_Data_String(0xc0,"20160127" );
  
  
 
 while(1)
 { 
   
   printf("A");
   
   switch(Push_count)
   {
   case 1:
     
    Lcd_Data_String(0x80,"1.      " );  // LCD 출력 
    Lcd_Data_String(0xc0,"Sensing " );
  
    while(Push_count==1)
    {
     Get_ADC( );
     
    }
     break;
     
   case 2:
  Lcd_Data_String(0x80,"2. Speed" );  // LCD 출력 
  
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
  Lcd_Data_String(0x80,"3. KP   " );  // LCD 출력
  
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
     
  Lcd_Data_String(0x80,"3.      " );  // LCD 출력 
  Lcd_Data_String(0xc0,"Start   " );
  
  

  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 
  TIM_Cmd(TIM4, ENABLE); 
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
  TIM_Cmd(TIM3, ENABLE);
  
  Delay_ms(500);
  
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //시스템 주기.
  TIM_Cmd(TIM2, ENABLE);
  
     
       

  
   while(Push_count==4)
    {
      
      //방지함수.
	if (percent[0]<10&&percent[1]<10&&percent[2]<10&&percent[3]<10 && percent[4]<10 && percent[5]<10 && percent[6]<10 && percent[7]<10 && percent[8]<10&& percent[9]<10&&percent[10]<10&&percent[11]<10&&percent[12]<10&&percent[13]<10 )
	{				
          TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);  //오른쪽 				
          TIM_Cmd(TIM3, DISABLE);				
          TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);  //왼쪽 				
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
         
        //시작가속-------- 시작버튼을 누르자마자 탈조나는 것을 방지한다. 
        if (speed_count == 0)
        {
          set_speed = set_speed + 10  ;
        
          if (set_speed>determined_speed)
	      speed_count=1;
        
        }
        //----------------
        
        //시작점 예외처리 
        
        if (Start_Delay == 1)			
        {	
          Start_count=Start_count+5;	//5ms마다 5씩 증가시킨다. 			
        }			
        if (Start_count == 1000)	        //1000ms가 지나면 들어간다. 		
        {				
          Start_Delay = 0;				
          Start_count = 0;	
         
        }
        
        //----------------
        
        //cross 예외처리---
        if ((percent[4]>80 && percent[5]>80 && percent[6]>80&&percent[7]>80&& percent[8]>80) || (percent[5]>80&&percent[6]>80&&percent[7]>80 && percent[8]>80 && percent[9]>80))			
        {	
          GPIOC->ODR = 0x0080;   //Buser On
          error = 0;					
          Cross_Delay = 1;			
        }
             		
        if (Cross_Delay == 1)			
        {			
          Cross_count=Cross_count+5;	//5ms마다 5씩 증가시킨다. 			
        }			
        if (Cross_count == Cross_Delay_time)	        //Cross_Delay_time=100ms가 지나면 들어간다. 		
        {				
          Cross_Delay = 0;				
          Cross_count = 0;	
          GPIOC->ODR = 0x0000;   //Buser Off
        }
        
        //-----------------
        
        //정지(감속)------엔드마크에서 정지한다.
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
     
  Lcd_Data_String(0x80,"4.      " );  // LCD 출력 
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
//ADC값 받는 함수 
void Get_ADC(void)
    {      
      for(num=0;num<14;num++)
      {
       GPIOD->ODR = LED[num];  //발광 on  
        
      ADC_RegularChannelConfig(ADC1, num,  1, ADC_SampleTime_13Cycles5);
      ADC_Cmd(ADC1,ENABLE);
      while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET); //변환이 끝나고 End of Conversion Stais가 활성화 되면 데이터를 가지고옴
      AD_Value[num]=(ADC1->DR)>>4; 
        
      GPIOD->ODR = 0x0000;  //발광 off
      
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

//무게중심법--------------------------------------------------------------------

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
//PID제어 함수

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
//RCC 설정----------------------------------------------------------------------
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
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); //PLL 설정8Mhz*9=72Mhz
    RCC_PLLCmd(ENABLE);
    
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != 0x08);
  }
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOC,ENABLE);//GPIO c와 e에 clock허용을 선언한다.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);// 타이머2 클럭허용
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD,ENABLE); //GPIO A와 D에 clock허용한다. 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//블루투스 UART1 clock허용.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);// 타이머4 클럭허용
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);// 타이머3 클럭허용
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);// ADC1클럭허용
 
}



//GPIO 설정---------------------------------------------------------------------
void Gpio_Initialize(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   
   
   //LCD Rs E
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;//핀 8번 9번 추가
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   
    //LCD
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;// 핀 1~7추가
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOE, &GPIO_InitStructure);
   
     //부저.
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //핀 7
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   
     //버튼
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;//핀 10,11,12번 추가 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   
     //블루투스 TX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//핀 9추가 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   //블루투스 RX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//핀 10추가 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //모터 TIM4(R)
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; //핀 8,9,10,11
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOE, &GPIO_InitStructure);
   
   
     //모터 TIM3(L)
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; //핀 12,13,14,15
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   
   //발광 
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;// 핀 1~14추가
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
   
     //ADC1  A포트 
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;// 핀 0~7   
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analog Input mode. 
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //ADC1  B포트  
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;// 핀 0,1   
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analog Input mode. 
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   
    //ADC1  C포트  
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;// 핀 0,1   
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analog Input mode. 
   GPIO_Init(GPIOC, &GPIO_InitStructure);
} 

// Nvic 함수선언----------------------------------------------------------
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
//10.11.12.13.14.15pin 에 대한...외부인트럽트를 사용하려면
//EXTI15_10_IRQChannel 선언을 하고



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
//외부인터럽트 설정. 
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
//시스템 주기 5ms.
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
//시스템 주기 5ms. 
void TIM2_Initialize(void)

{   
   
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

   TIM_TimeBaseStructure.TIM_Period = 120; //1~65535  
   TIM_TimeBaseStructure.TIM_Prescaler = 300;     
   //시간 계산 법 = 36MHZ 공급받는다.36000,000  x 1/period  x 1/prescaler  [1200, 300=   10ms]
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  
   TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE); //TIM IT enable
   TIM_Cmd(TIM2, DISABLE); //TIM2 enable counter 

}

//Printf를 사용하기 위한 정의---------------------------------------------------
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

void UART1_Initialize(void){ //PCLK2를 사용한다. 
  
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   
   
   USART_InitStructure.USART_BaudRate = 115200; 
   // baud rate= 데이터 통신에서 직렬 전송의 변조 속도를 1초간에 전송되는 신호의 수로 나타낸 값.
   
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   //8비트 또는 9비트로 설정 할 수있다. 
   
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   //하나의 단어(word)의 끝을 표시하기 위해 최후에 부가하는 2비트(1 1.5 2 로 설정 가능)
   
   USART_InitStructure.USART_Parity = USART_Parity_No;
   //1”의 비트의 개수가 짝수인지 홀수인지를 결정

   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   // 수신측에서 수신불가 상태인 경우 송신측에서 데이터 전송하지 않도록 하고 다시 수신가능 상태가 되었을 때만 데이터 전송하는 방식. 
   
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   //USART_CR1 레지스터에서 bit3 TE Transmitter enable 과 Bit 2 RE Receiver enable 를 설정하기 위함. 해당비트위치가 1이 되도록함. 
   
   USART_Init(USART1, &USART_InitStructure);
   //변수선언하고 0으로 초기화. 
   
   USART_Cmd(USART1, DISABLE); // 사용하려면 ENABLE 해줘야됨 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
   //USART1 Enable
   
   
   NVIC_InitStructure.NVIC_IRQChannel = 37;  //USART1_IRQChannel
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
}


//(BLUETOOTH 통신)USART1 인터럽트 설정------------------------------------------
void USART1_IRQHandler(void)
//UART1으로 수신된 값을 읽는 것. 
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)// 비트에 0이 아닐 경우에 데이터가 수신.
    {         
       
        control_data  = USART_ReceiveData(USART1); //0이 아닐 경우에 데이터를 읽는다. control_data= 버퍼에 값을 저장하는 역할을 하는 변수.
        
        /*Buffer룰 구현해야하는 이유.
        
        수신시간에비해 처리시간이 길어서 인터럽트 발생부분에서는 버퍼에 단순히 저장만하고 처리는 다른곳에서 한다.
        */
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
        // pending bit를 clear.(안해주면 인터럽트가 처리되었는지 알수없고 다시 인터럽트가 발생한것으로 인지해서 계속 핸들러 호출) 
       
    }
    
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)//비트에 0이 아닐 경우에 데이터가 송신 
    {         
     
        USART_SendData(USART1,TxBuf[TxOutCnt]);
        
        if(TxOutCnt<BUFFER_SIZE-1) TxOutCnt++; // Txoutcount가 buffer size 보다 작으면 +1씩 샌다. 
        else TxOutCnt = 0;      
            
        if(TxOutCnt == TxInCnt)// Txoutcount가 Txincount가 되면 tx 끄고 RX 킨다. 
        {
          USART_ITConfig(USART1, USART_IT_TXE, DISABLE); 
          USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        }         
    }        
   
}

void TIM4_Initialize(void)//모터  R

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
void TIM4_IRQHandler(void)//모터 R

{

  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  
  TIM4 ->ARR =count(R_speed);  
  
// GPIOE->ODR=(GPIOE->ODR&0xf0ff)|mot_phs_R[Phase_R];
 GPIOE->ODR= mot_phs_R[Phase_R];


  Phase_R++;
  

  if(Phase_R==8)Phase_R=0;
  
  

} 

//------------------------------------------------------------------------------

void TIM3_Initialize(void)//모터 L

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
void TIM3_IRQHandler(void)//모터 L

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
   //ADC를 독립모드로 사용. single mode 
   ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
   //비트를 0으로 만들면 DISABLE, 비트를 1로 만들면 ENABLE
   //ENABLE하게 되면 채널들을 스캔하게된다. 각각채널에서 conversion을 수행한다. 
   ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
   //0으로하면 ADC변환을 한번만 수행, 1로 설정하면 반복해서 수행.
   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
   //regular채널에 대한 conversion을 시작하도록하는 trigger를 지정 할 수 있는 것. 
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
   //16비트의 데이타 레지스터에 어떻게 저장할 것인가?  
   ADC_InitStructure.ADC_NbrOfChannel = 14; 
   //변환할 채널수 14개. 
   ADC_Init(ADC1, &ADC_InitStructure);
   //ADC_TempSensorVrefintCmd(ENABLE); //온도센서. 
   /*
   ADC_RegularChannelConfig(ADC1, ADC_Channel_0,  1, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_1,  2, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_13Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_13Cycles5);
   // 전압을 샘플링하는 싸이클의 수. convertion 시간= sampling time +
   */
   
   ADC_Cmd(ADC1, ENABLE); //Enable   ADC1 
   
   
   ADC_ResetCalibration(ADC1);//측정 오차를 줄일 수있다. 
   while(ADC_GetResetCalibrationStatus(ADC1));//0으로 바뀔때까지 기다림. 

   ADC_StartCalibration(ADC1);
   while(ADC_GetCalibrationStatus(ADC1)); 
  
   
}