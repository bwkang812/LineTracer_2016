#ifndef _Lcd_H_
#define _Lcd_H_
void Lcd_Data_Clock(u8 Data);
void Lcd_Option_Clock(u8 Data);
void Lcd_Initialize(void);
void Lcd_Data_String(u8 Address,u8 Data[8]);
void Lcd_Motor_Test(u16 Motot_Speed);
void Lcd_Sensor_Test(u8 Address,u8 Data);
void Lcd_Posion_Test(float Data);
void Percent(u8 Data);
void Lcd_Range(s16 Data);
#endif