#include "stm32f10x.h"
#include "Delay.h"
#define E GPIO_Pin_8
#define RS GPIO_Pin_9
void Lcd_Option_Clock(u16 Data)//초기화 클럭주기 
{
   GPIOC->BRR = RS; //0
   GPIOE->ODR &= 0x00;
   GPIOE->ODR |= Data; Delay_us(10);
   GPIOC->BSRR = E; Delay_us(1); //1
   GPIOC->BRR = E; Delay_us(50); //0
}
void Lcd_Data_Clock(u8 Data)//초기화 클럭주기 
{
   //Data=Data<<8;
   GPIOC->BSRR = RS; //1
   GPIOE->ODR &= 0x00;
   GPIOE->ODR |= Data; Delay_us(10);
   GPIOC->BSRR = E;Delay_us(1); //1
   GPIOC->BRR = E; Delay_us(50); //0
}
void Lcd_Initialize(void)//LCD초기화
{
  Delay_ms(10);
  Lcd_Option_Clock(0x38);Delay_ms(5);
  Lcd_Option_Clock(0x38);Delay_ms(1);
  Lcd_Option_Clock(0x38);Delay_ms(5);
  Lcd_Option_Clock(0x0c);//display
  Lcd_Option_Clock(0x06);//entery
  Lcd_Option_Clock(0x14);//entery
  Lcd_Option_Clock(0x01);Delay_ms(5);
}
void Lcd_Data_String(u8 Address,u8 Data[8])
{
   u8 Count;
   Lcd_Option_Clock(Address);
   for(Count=0; Count<8; Count++)
   Lcd_Data_Clock(Data[Count]);
}
void Lcd_Motor_Test(u16 Motot_Speed)
{
   Lcd_Option_Clock(0xc1);
   Lcd_Data_Clock(0x30+(Motot_Speed/1000));
   Lcd_Option_Clock(0xc3);
   Lcd_Data_Clock(0x30+((Motot_Speed%1000)/100));
   Lcd_Data_Clock(0x30+((Motot_Speed%100)/10));
}
void Lcd_Sensor_Test(u8 Address,u8 Data)
{
   Lcd_Option_Clock(0xc1);
   Lcd_Data_Clock(0x30+(Address/10));
   Lcd_Data_Clock(0x30+(Address%10));
   Lcd_Option_Clock(0xc4);
   Lcd_Data_Clock(0x30+(Data/100));
   Lcd_Data_Clock(0x30+((Data%100)/10));
   Lcd_Data_Clock(0x30+(Data%10));
}
void Lcd_Posion_Test(float Data)
{
   Lcd_Option_Clock(0xc2);
   if(Data<0)Lcd_Data_Clock(0x2d),Data=Data*-1;
   else if(Data==0)Lcd_Data_Clock(0x10);
   else if(Data>0)Lcd_Data_Clock(0x2b);
   Lcd_Data_Clock(0x30+((int)Data/1000));
   Lcd_Data_Clock(0x30+((int)Data%1000)/100);
   Lcd_Data_Clock(0x30+((int)Data%100)/10);
   Lcd_Data_Clock(0x30+(int)Data%10);
}
void Lcd_Range(s16 Data)
{
   Lcd_Option_Clock(0xc3);
   Lcd_Data_Clock(0x30+((int)Data/1000));
   Lcd_Data_Clock(0x30+((int)Data%1000)/100);
   Lcd_Data_Clock(0x30+((int)Data%100)/10);
   Lcd_Data_Clock(0x30+(int)Data%10);
}
void Percent(u8 Data)
{
  Lcd_Option_Clock(0xc4);
  Lcd_Data_Clock(0x30+(Data/10));
  Lcd_Data_Clock(0x30+(Data%10));
}