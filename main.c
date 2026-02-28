/*****************凉介单片机设计******************
                      STM32                
 * 版本     :  V1.0
 * MCU      :  STM32F103C8T6
 * B站      :  凉介单片机设计
 * 作者     :  凉介
 * 微信     :  danpianji6
  联系我免费获取更多开源资料！单片机定做！
**********************BEGIN***********************/
#include "stm32f10x.h"
#include "usart.h"
#include "myiic.h"
#include "MAX30102.h"
#include "algorithm.h"
#include "oled.h"
#include "dht11.h"
#include "esp8266.h"
#include "string.h"
#include "mq4.h" //MQ4天然气传感器驱动库
#include "led.h"
#include "key.h"
#include "gps.h"
#include "hc.h"
#include <timer.h>
#include "gy906.h"
#include "rtc.h"
#include "mpu6050.h"
#include "Store.h"
#include "tcs34725.h"
#include "adc.h"
u8 color_detect_enable = 0; // 颜色检测使能标志位
u8 last_color = 0; // 0:无色 1:红 2:绿 3:黄
COLOR_RGBC rgb;
COLOR_HSL  hsl;
 u16 Light;//灯光
/*MPU6050参数*/
extern u8 MPU_flag;
extern u8 QX_Flag,SD_Time,SD_Flag;
/*MPU6050参数*/
void Title(void);
u16 Light_Check(void);
void Page_1(void);
void Page_4(void);
void Page_5(void);
void heartrate_check(void);
void WIFIChoose_Page(void);
void SU_03T(void);
signed short int temperature_[BufferSize] = {0x0000,0x0000,0x0000,0x0000,0x0000};
int num=0;
int temp=0;


/*********心率参数**************/
#define MAX_BRIGHTNESS 255
uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t heartrate;   //心率
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
extern double lon;//经度
extern double lat;//纬度
//dht11添加变量
DHT11_Data_TypeDef DHT11_Data;
extern  DHT11_Data_TypeDef DHT11_Data;
extern _calendar_obj calendar;//时钟结构体 
u8 keynum;//按键值

	extern		uint16_t Light;
		extern		uint16_t xx;
extern char str1[20];

char data[200];
u8 page=0;	
u8 length,alarm_length=5;
/*心率参数*/
int i;
float f_temp;
uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
int32_t n_brightness;
/*心率参数*/
 
 u8 row=2;//阈值设置的光标Y坐标
extern u8 GPS_Success;
extern u8 ESP_Success_First;//第一次连接WIFI成功标志位
u8 HeartRate_Flag=0;
u8 count_Light=20,count_Length=5,count_Temp=39,count_Hum=90,count_HeartRate=150;
extern	u8 Res1;
uint8_t Set_CY_hour,Set_CY_min;
u8 Alarm=0x00;
int main(void)
{		

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//中断控制器分组设置
  Usart1_Init(9600);							//串口1，打印信息用
  Usart2_Init(115200);						//串口2，驱动ESP8266用
	USART3_Init(9600);
	Store_Init();				//参数存储模块初始化，在上电的时候将闪存的数据加载回Store_Data，实现掉电不丢失
	count_Length=Store_Data[1];
	Set_CY_hour=Store_Data[2];
	Set_CY_min=Store_Data[3];
	TIM3_Int_Init(10-1,7200-1);//10Khz的计数频率，计数到5000为500ms  
  OLED_Init();
  OLED_Clear(0);
  LED_Init();
	Color_LED_Init();
	Vibrate_Init();
	BEEP_Init();
  Key_Init();
	Hcsr04Init();
	TCS34725_Init();
  DHT11_Init();           //DHT11初始化 
	RTC_Init();
 	MPU6050_Init();//陀螺仪初始化
	MPU6050_EXTI_Init();
	Adc_Init();
 do {
			keynum=Key_GetNum();
			WIFIChoose_Page();
			if(keynum==1)//按键1
			{
				// 执行联网操作
				OLED_Clear(0); 
				Title();
				ESP8266_Init();
			//	WIFIUpdataFlag=1;//允许ESP8266数据上传标志
				break; // 退出循环
			} 
			else if (keynum ==2)//按键2
			{
				OLED_Clear(0); // 不执行任何操作，直接退出循环
				break; // 退出循环
			}
			else 
			{
			}
		} while (1); // 一直循环直到用户选择了1或2

	//ESP8266_Init();
	//Hcsr04Init();
//  while(1)
//{
//	temperature_[num]=get_temperature();
//	OLED_ShowNum(50,0,change(temperature_[num])+3.14,2,16);//加3.14是因为我没有聚集棱镜所以辐射无法聚集导致测温有误差属于修正误差的额外加值(可以自己修改)
//}


  while(1)
  {		

		
	  SU_03T();
		if(MPU_flag==1)//MPU中断标志位
		{
			MPU_Data();
			MPU_flag=0;
		}
		Light=Light_Check();
		
		WIFI_ReConnect();//WIFI断开重连函数
		TCS34725_GetRawData(&rgb);
		RGBtoHSL(&rgb,&hsl);
	  length=Hcsr04GetLength();
		display_lon_lat();
		keynum=Key_GetNum();
		
		if(keynum==1)//
		{  
			OLED_Clear(0);
			page++;                                              
			if(page==4){ page=0;}
		}

		if(page==0)  
		{
			Page_1();
		}
		if(page==1)
		{
			Title();
			OLED_ShowCHinese(0, 4, 101);//光照
			OLED_ShowCHinese(16,4, 102);//
			OLED_ShowString(32,	4,(u8*)":",16); 
			OLED_ShowNum(40,		4,Light,3,16);
			OLED_ShowString(64,	4,(u8*)"%",16);
			// 添加颜色判断逻辑
			OLED_ShowCHinese(0, 2, 81);//颜色
			OLED_ShowCHinese(16,2, 82);//
			OLED_ShowString(32,	2,(u8*)":",16); 
        // 颜色检测和语音提示逻辑
        if(keynum == 3)
				{ // 按键3控制颜色检测功能开关
   //         color_detect_enable = ~color_detect_enable;
            if(color_detect_enable)
						{
               SU_03T_SendBegin();
               USART_SendData(USART1, 0x0E); // 发送开启颜色检测提示音
               SU_03T_SendEnd();
								color_detect_enable=0;
						}
						else
						{
                SU_03T_SendBegin();
                USART_SendData(USART1, 0x0D); // 发送关闭颜色检测提示音
                SU_03T_SendEnd();
							color_detect_enable=1;
            }
        }
            if(color_detect_enable)
						{
							Color_LED=1;
							OLED_ShowCHinese(96, 2, 109);//开启
							OLED_ShowCHinese(112,2, 110);//
						}
						else
						{
							
							Color_LED=0;
							OLED_ShowCHinese(96, 2, 111);//关闭
							OLED_ShowCHinese(112,2, 112);//
							OLED_ShowString(40,	2,(u8*)"  ",16); 
            }
		if(color_detect_enable) 
		{
			if(hsl.s < 15) 
			{  // 饱和度很低时显示白色
					OLED_ShowCHinese(40, 2, 76);//白
					last_color = 0;
			} 
			else if(hsl.l < 15) 
			{  // 亮度很低时显示黑色
					OLED_ShowCHinese(40, 2, 77);//黑
					last_color = 0;
			} 
			else 
			{
				// 基于色相角度判断具体颜色
        if(hsl.h < 15 || hsl.h >= 345) {
            OLED_ShowCHinese(40, 2, 83);  // "红"
        } else if(hsl.h < 75) {
            OLED_ShowCHinese(40, 2, 85);  // "黄"
        } else if(hsl.h < 165) {
            OLED_ShowCHinese(40, 2, 84);  // "绿"
        } else {
            if(hsl.h < 195) {
                OLED_ShowCHinese(40, 2, 79);  // "青"
            } else if(hsl.h < 285) {
                OLED_ShowCHinese(40, 2, 86);  // "蓝"
            } else if(hsl.h < 345) {
                OLED_ShowCHinese(40, 2, 87);  // "紫"
            }
        }
			}
			
		}
		
			OLED_ShowCHinese(0, 6, 74);//灯光
			OLED_ShowCHinese(16,6, 75);//
			OLED_ShowString(32,	6,(u8*)":",16);
			if(LEDState==0x00)
			{
				OLED_ShowCHinese(40, 6, 57);//关
			}
			else
			{
				OLED_ShowCHinese(40, 6, 56);//开
			}
		if(keynum==2)//
		{  
				LEDState=~LEDState;
				if(LEDState==0x00)
				{
					LED_OFF();
					sprintf((char*)data,"%s&msg=LED_F\r\n",Secret_Key);
					ESP8266_SendData((unsigned char *)data);

				}
				if(LEDState==0xff)
				{
					LED_ON();
					sprintf((char*)data,"%s&msg=LED_T\r\n",Secret_Key);
					ESP8266_SendData((unsigned char *)data);
				}
		}
			
		}
		if(color_detect_enable) 
		{
// 添加颜色检测和语音提示逻辑
        if(hsl.s < 15 || hsl.l < 15)
				{  // 灰色或黑色
            last_color = 0;
        } 
				else
				{
            // 根据色相角度判断颜色
            if(hsl.h < 15 || hsl.h >= 345)
						{  // 红色
                if(last_color != 1) {
                    SU_03T_SendBegin();
                    USART_SendData(USART1, 5);  // 发送红色语音信息
                    SU_03T_SendEnd();
                    last_color = 1;
                }
            } 
						else if(hsl.h < 75) 
						{  // 黄色
                if(last_color != 2) 
								{
                    SU_03T_SendBegin();
                    USART_SendData(USART1, 10);  // 发送黄色语音信息
                    SU_03T_SendEnd();
                    last_color = 2;
                }
            } 
						else if(hsl.h < 165)
						{  // 绿色
                if(last_color != 3) 
								{
                    SU_03T_SendBegin();
                    USART_SendData(USART1, 7);  // 发送绿色语音信息
                    SU_03T_SendEnd();
                    last_color = 3;
                }
            } 
						else 
						{  // 其他颜色
                last_color = 0;
            }
        }	
		}
           if(color_detect_enable)
						{
							Color_LED=1;
						}
						else
						{
							Color_LED=0;

            }
		if(page==2)
		{
			Page_2();
			if(GPS_Success==0&&ESP_Success_First==1)//避免GPS的时间和RTC时间冲突，避免非联网模式无法更新当前时间
			{
				OLED_ShowNum(0,		6,calendar.w_year,4,16);
				OLED_ShowString(32,	6,(u8*)"-",16);
				OLED_ShowNum(40,	6,calendar.w_month,2,16);
				OLED_ShowString(56,	6,(u8*)"-",16);
				OLED_ShowNum(64,	6,calendar.w_date,2,16);
				
				OLED_ShowNum(0+64+24,		6,calendar.hour,2,16);
				OLED_ShowString(16+64+24,	6,(u8*)":",16);
				OLED_ShowNum(24+64+24,		6,calendar.min,2,16);

			}
		}
		if(page==3)
		{
			Page_5();
		}
/******自动开灯******/
		if(Light<30)
		{
			if(LEDState==0x00)
			{
				LED_OFF();
				sprintf((char*)data,"%s&msg=LED_T\r\n",Secret_Key);
				ESP8266_SendData((unsigned char *)data);
        SU_03T_SendBegin();
        USART_SendData(USART1, 0x0C);  // 发送开灯语音消息
        SU_03T_SendEnd();
			}
			LEDState=0xff;
			LED_ON();
		}
//		if(page==4)
//		{
//			Page_4();
//		}
    /*向巴法云发送数据***************************************/
    sprintf((char*)data,"cmd=2&uid=66e8c21f247d44aeba92be4cccf55956&topic=DaoMangZhang&msg=#%d.%d#%d.%d#%f#%f#%d#%d#%d#%d#%d#\r\n",DHT11_Data.temp_int,0,DHT11_Data.humi_int,DHT11_Data.humi_deci,lon,lat,length,count_Length,QX_Flag,Alarm,Light);
    ESP8266_SendData((unsigned char *)data);
 
		
    /*读取温湿度数据***************************************/
    Read_DHT11(&DHT11_Data);								


	if(length<count_Length||QX_Flag==1)
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		VIBRATE=1;
		
	}
	else
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
		VIBRATE=0;
	}

     

  }
}
u8 flag_CY;
void SU_03T(void)
{
	
		if((calendar.hour==Set_CY_hour&&calendar.min==Set_CY_min&&calendar.sec==00))//MPU中断标志位
		{
			SU_03T_SendBegin();
			USART_SendData(USART1,9);
			Delay_ms(1);
			USART_SendData(USART1,calendar.w_year);
			Delay_ms(1);
			USART_SendData(USART1,calendar.w_month);
			Delay_ms(1);
			USART_SendData(USART1,calendar.w_date);
			Delay_ms(1);
			USART_SendData(USART1,calendar.hour);
			Delay_ms(1);
			USART_SendData(USART1,calendar.min);
			Delay_ms(1);
			USART_SendData(USART1,calendar.sec);
			SU_03T_SendEnd();
			Delay_ms(500);//防止接收太快二次朗读。
			
		}

		  	if(Res1==0x01)
		{	
			
			SU_03T_SendBegin();
			USART_SendData(USART1,11);//温度的指令
			Delay_ms(1);
			USART_SendData(USART1,DHT11_Data.temp_int);
			Delay_ms(1);
			USART_SendData(USART1,DHT11_Data.temp_deci);
			SU_03T_SendEnd();
			Res1=0;
			
		}
		else if(Res1==0x02)
			{	
				SU_03T_SendBegin();
				USART_SendData(USART1,02);//湿度的指令
				Delay_ms(1);
				USART_SendData(USART1,DHT11_Data.humi_int);
				SU_03T_SendEnd();
				Res1=0;
				
			}
		else if(Res1==0x03)//询问时间
		{
			SU_03T_SendBegin();
			USART_SendData(USART1,06);
			Delay_ms(1);
			USART_SendData(USART1, calendar.w_year & 0xFF);         // 发送低8位
			Delay_ms(1);
			USART_SendData(USART1, (calendar.w_year >> 8) & 0xFF);  // 发送高8位
			Delay_ms(1);
			USART_SendData(USART1,00);
			Delay_ms(1);
			USART_SendData(USART1,00);
			Delay_ms(1);
			USART_SendData(USART1,calendar.w_month);
			Delay_ms(1);
			USART_SendData(USART1,calendar.w_date);
			Delay_ms(1);
			USART_SendData(USART1,calendar.hour);
			Delay_ms(1);
			USART_SendData(USART1,calendar.min);
			Delay_ms(1);
			USART_SendData(USART1,calendar.sec);
			SU_03T_SendEnd();
			Res1=0;

		}
		else if(Res1==0x04)//出门播报的指令
		{
			SU_03T_SendBegin();
			USART_SendData(USART1,8);//出门播报的指令
			Delay_ms(1);
			USART_SendData(USART1,DHT11_Data.temp_int);
			Delay_ms(1);
			USART_SendData(USART1,DHT11_Data.temp_deci);
			SU_03T_SendEnd();
			Res1=0;

		}
		else if(Res1==0x05)//询问距离
		{
			SU_03T_SendBegin();
			USART_SendData(USART1,1);//询问距离的指令
			Delay_ms(1);
			USART_SendData(USART1,length);
			SU_03T_SendEnd();
			Res1=0;

		}
		else if(Res1==0x06)//打开灯光
		{
			LEDState=0xff;
			LED_ON();
			Res1=0;
		}
		else if(Res1==0x07)//关闭灯光
		{
			LEDState=0x00;
			LED_OFF();
			Res1=0;
		}
		else if(Res1==0x08)//开启颜色识别
		{
			color_detect_enable=1;
			Res1=0;
		}
		else if(Res1==0x09)//关闭颜色识别
		{
			color_detect_enable=0;
			Res1=0;
		}
}

u8 pageNum=1;
void Page_5(void)
{

		OLED_ShowCHinese(26, 0,88);//阈值设置
		OLED_ShowCHinese(44, 0,89);//
		OLED_ShowCHinese(62, 0,90);//
		OLED_ShowCHinese(80, 0,91);//
		OLED_ShowString(110,row,(u8*)"<",16); 

		OLED_ShowString(110,row,(u8*)"<",16);  
		OLED_ShowCHinese(0, 2, 99);//距离
		OLED_ShowCHinese(16,2, 100);//
		OLED_ShowCHinese(32,2,88);//阈值
		OLED_ShowCHinese(48,2,89);//
		OLED_ShowString(32+32,	2,(u8*)":",16);	
		OLED_ShowNum(40+32,		2,count_Length,2,16);

		OLED_ShowCHinese(0, 	4, 103);//吃药时间
		OLED_ShowCHinese(16,	4, 104);//
		OLED_ShowCHinese(32,	4,105);//
		OLED_ShowCHinese(48,	4,106);//
		OLED_ShowString(64,		4,":",16);		 
		OLED_ShowNum(72,		4,Set_CY_hour,2,16);
		OLED_ShowString(72+16,	4,":",16);	
		OLED_ShowNum(72+24,		4,Set_CY_min,2,16);
		


			if(keynum==3)//按键3
			{
				count_Length++;
				Store_Data[1] = count_Length;
				Store_Save();// 只在需要时执行 Flash 写入
			}
			if(keynum==4)//按键4
			{
				count_Length--;
				Store_Data[1] = count_Length;
				Store_Save();// 只在需要时执行 Flash 写入
			}

}
void Title(void)
{
	  OLED_ShowCHinese(8,  0,15);//智能导盲系统
    OLED_ShowCHinese(26, 0,16);//
    OLED_ShowCHinese(44, 0,45);//
    OLED_ShowCHinese(62, 0,46);//
    OLED_ShowCHinese(80, 0,19);//
    OLED_ShowCHinese(98, 0,20);//
}
void Page_1(void)
{
	 /*******************OLED显示数据***************************************/
		Title();
	//显示温度
    OLED_ShowCHinese(0, 2,10);//温
    OLED_ShowCHinese(18,2,12);//度
    OLED_ShowCHinese(34,2,13);//：
    OLED_ShowNum(42,2,DHT11_Data.temp_int,2,16);
    OLED_ShowString(58,2,(u8*)".",16); 
    OLED_ShowNum(66,2,0,1,16);
    OLED_ShowCHinese(80,2,25); 
        
    OLED_ShowCHinese(0, 4,11);//湿
    OLED_ShowCHinese(18,4,12);//度
    OLED_ShowCHinese(34,4,13);//：
    OLED_ShowNum(42,4,DHT11_Data.humi_int,2,16);
    OLED_ShowString(58,4,(u8*)".",16); 
    OLED_ShowNum(66,4,DHT11_Data.humi_deci,1,16);
    OLED_ShowString(80,4,(u8*)"%",16);  
	  
	OLED_ShowCHinese(0, 6, 99);//距离
    OLED_ShowCHinese(16,6, 100);//
    OLED_ShowString(32,	6,(u8*)":",16); 
	OLED_ShowNum(40,	6,length,3,16);
    OLED_ShowString(64,	6,(u8*)"cm",16);  
		if(keynum==2)//按键2
		{
			Alarm=~Alarm;
			if(Alarm==0xff)
			{
				OLED_ShowCHinese(96, 6, 107);//报警
				OLED_ShowCHinese(112, 6, 108);//
			}
			else
			{
				OLED_ShowString(96,	6,(u8*)"    ",16);
			}
		}
		 

}

void WIFIChoose_Page(void)
{
	Title();
	
	OLED_ShowCHinese(8-8,  	3,92);//请选择是否联网
	OLED_ShowCHinese(26-8, 	3,93);//
	OLED_ShowCHinese(44-8, 	3,94);//
	OLED_ShowCHinese(62-8, 	3,95);//
	OLED_ShowCHinese(80-8, 	3,96);//
	OLED_ShowCHinese(98-8, 	3,97);//
	OLED_ShowCHinese(114-8, 3,98);//
	
	OLED_ShowString(16,		6,(u8*)"1.",16);
	OLED_ShowCHinese(32, 	6,95);//
	OLED_ShowString(64,		6,(u8*)"2.",16);
	OLED_ShowCHinese(80,	6,96);//
}
u16 Light_Check(void)
{
	u16 light;float light_AD;
	light_AD=4096-Get_Adc_Average(1,1);
	light=light_AD/4095*100;

	return light;
}