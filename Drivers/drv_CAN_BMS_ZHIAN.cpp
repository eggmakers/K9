#include "drv_CAN_BMS_ZHIAN.hpp"
#include "drv_CAN.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"

//本机地址
#define MY_ID 0x1
//电池默认地址
#define BAT_ID 0xA

//帧定义
#define MSG_HEARTBEAT 		0x1		//心跳包
#define MSG_BATSTATUS 		0x2		//实时信息
#define MSG_BATIDENTITY		0x3		//身份信息
#define MSG_BASICS			0x4		//基本信息
#define MSG_DETECT			0x5		//监测信息

//id		//	S起始为1	E结束为1	N包序号
#define ID_BATSTATUS(S,E,N) 	(0x0 << 26) | (MSG_BATSTATUS << 20) | (0x7 << 17) | (MY_ID << 11) | (BAT_ID << 5) | (S << 4) | (E << 3) | (N << 0)
#define ID_DETECT(S,E,N) 		(0x0 << 26) | (MSG_DETECT << 20) | (0x7 << 17) | (MY_ID << 11) | (BAT_ID << 5) | (S << 4) | (E << 3) | (N << 0)

// CRC高位字节值表 
const uint8_t auchCRCHi[] = 
{ 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40 
}; 

// CRC 低位字节值表 
const uint8_t auchCRCLo[] =
{ 
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 
    0x41, 0x81, 0x80, 0x40 
};

// puchMsg:要校验的数组 
// usDataLen:校验的长度 
//返回值:CRC16码 
static uint16_t Get_Crc16(uint8_t *puchMsg, uint16_t usDataLen) 
{ 
    uint8_t uchCRCHi = 0xFF; //高CRC 字节初始化 
    uint8_t uchCRCLo = 0xFF; //低CRC 字节初始化 
    uint32_t uIndex;         // CRC 循环中的索引 
    while (usDataLen--)      //传输消息缓冲区 
    { 
        uIndex = uchCRCLo ^ *puchMsg++; //计算CRC 
        uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex]; 
	} 
	return (uchCRCHi << 8 | uchCRCLo); 
} 


struct DriverInfo
{
	CanMailBox* mail_box;
	uint32_t sensor_key;
};

typedef struct
{   //电池实时信息
	uint16_t crc16;				//CRC校验
	uint16_t bus_voltage;		//总电压		10mv
	int32_t bus_current;		//总电流		ma		充电为正放电为负
	int16_t bat_temp;			//电芯温度		0.1℃
	int16_t mos_temp;			//MOSFET温度	0.1℃
	uint16_t bat_capacity;		//电量百分比	0.1%
	uint32_t bat_status;		//电池组状态	
	uint16_t bat_series_num;	//电池串联数量	
	uint16_t bat_voltage[64];	//电芯电压		mv
}__PACKED _BatStatusMessage;

typedef struct
{   //电池基本信息
	uint16_t batteryCapacityExpected;		//电池设计容量	单位（100mAh）
	uint16_t battery_C_ratel;				//电池放电倍率	C
	uint16_t batteryNominalVoltage;			//标称电压
	uint16_t battery_series_num;			//电池串联数量	
}__PACKED _BatBasicsMessage;

typedef struct
{   //电池监测信息
	uint16_t crc16;					//CRC校验
	uint16_t health;				//健康度	%
	uint16_t useCircleCount;		//循环次数
	uint16_t overchargeCount;		//过充次数
	uint16_t overfallCount;			//过放次数
	uint16_t overcurrentCount;		//过流次数
	uint16_t overtempratureCount;	//过温次数
	uint16_t socCount;				//Soc值
	uint16_t sohCount;				//Soh值
	uint16_t chargeCapacity;		//充电容量		单位（100mAh）
	uint16_t batteryCapacityRemain;	//电池剩余容量 	单位（100mAh）
	uint16_t batteryInternalRes;	//电池内阻值	单位（MR）
}__PACKED _BatDetectMessage;		

static inline void send_Basics( CanMailBox* mailBox )
{
	CanPacket mail;
	mail.IdType = 1;
	mail.DataLength = 8;
	mail.FrameType = 0;
	mail.FDFormat = 0;
	mail.Identifier = (0x0 << 26) | (MSG_BASICS << 20) | (0x7 << 17) | (BAT_ID << 11) | (MY_ID << 5) | (1 << 4) | (1 << 3) | (0 << 0);
	
	struct PACKET
	{
		uint32_t pre_regged_mask;
		uint32_t regged_mask;
	}__PACKED;
	PACKET* packet = (PACKET*)(mail.data);
	packet->pre_regged_mask = 0;
	packet->regged_mask = 0;
	
	mailBox->SendMail(mail);
}

static inline void send_Detect( CanMailBox* mailBox )
{
	CanPacket mail;
	mail.IdType = 1;
	mail.DataLength = 8;
	mail.FrameType = 0;
	mail.FDFormat = 0;
	mail.Identifier = (0x0 << 26) | (MSG_DETECT << 20) | (0x7 << 17) | (BAT_ID << 11) | (MY_ID << 5) | (1 << 4) | (1 << 3) | (0 << 0);
	
	struct PACKET
	{
		uint32_t pre_regged_mask;
		uint32_t regged_mask;
	}__PACKED;
	PACKET* packet = (PACKET*)(mail.data);
	packet->pre_regged_mask = 0;
	packet->regged_mask = 0;
	
	mailBox->SendMail(mail);
}

static void CAN_Radar_BMS_ZHIAN_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	_BatStatusMessage bat_status_message;
	uint8_t status_buff[64];
	
	_BatBasicsMessage bat_basics_message;
	
	_BatDetectMessage bat_detect_message;
	uint8_t detect_buff[64];
	
	TIME heartBeat_TIME(false);
	while(1)
	{	
		if( heartBeat_TIME.get_pass_time() > 1 )
		{	// 发送基本信息
			heartBeat_TIME = TIME::now();
			send_Basics(driver_info.mail_box);
			send_Detect(driver_info.mail_box);
		}
		
		CanPacket mail;
		if( driver_info.mail_box->receiveMail( &mail, 0.5 ) )
		{
			switch(mail.Identifier)
			{
				case (0x0 << 26) | (MSG_BASICS << 20) | (0x7 << 17) | (MY_ID << 11) | (BAT_ID << 5) | (1 << 4) | (1 << 3) | (0 << 0):
				{	// 电池基本信息
					memcpy((uint8_t *)&bat_basics_message, (uint8_t *)&mail.data[0], 8);
					break;
				}
				case ID_DETECT(1,0,1):
				{	// 电池监测信息帧1
					memcpy((uint8_t *)&detect_buff[0], (uint8_t *)&mail.data[0], 8);
					break;
				}
				case ID_DETECT(0,0,2):
				{	// 电池监测信息帧2
					memcpy((uint8_t *)&detect_buff[8], (uint8_t *)&mail.data[0], 8);
					break;
				}
				case ID_DETECT(0,1,3):
				{	// 电池监测信息帧3
					memcpy((uint8_t *)&detect_buff[16], (uint8_t *)&mail.data[0], 8);
					
					uint16_t crc16 = Get_Crc16((uint8_t *)&detect_buff[2], 22);
					if(crc16 == (detect_buff[0] | (detect_buff[1] << 8))) 
					{	//crc校验通过
						memcpy((uint8_t *)&bat_detect_message, (uint8_t *)&detect_buff[0], 24);
					}
					break;
				}
				case ID_BATSTATUS(1,0,1):
				{	// 电池运行信息帧1
					memcpy((uint8_t *)&status_buff[0], (uint8_t *)&mail.data[0], 8);
					break;
				}
				case ID_BATSTATUS(0,0,2):
				{	// 电池运行信息帧2
					memcpy((uint8_t *)&status_buff[8], (uint8_t *)&mail.data[0], 8);
					break;
				}
				case ID_BATSTATUS(0,0,3):
				{	// 电池运行信息帧3
					memcpy((uint8_t *)&status_buff[16], (uint8_t *)&mail.data[0], 8);
					break;
				}
				case ID_BATSTATUS(0,0,4):
				{	// 电池运行信息帧4
					memcpy((uint8_t *)&status_buff[24], (uint8_t *)&mail.data[0], 8);
					break;
				}
				case ID_BATSTATUS(0,0,5):
				{	// 电池运行信息帧5
					memcpy((uint8_t *)&status_buff[32], (uint8_t *)&mail.data[0], 8);
					break;
				}
				case ID_BATSTATUS(0,0,6):
				{	// 电池运行信息帧6
					memcpy((uint8_t *)&status_buff[40], (uint8_t *)&mail.data[0], 8);
					break;
				}
				case ID_BATSTATUS(0,1,7):
				{	// 电池运行信息帧7
					memcpy((uint8_t *)&status_buff[48], (uint8_t *)&mail.data[0], 8);
					
					uint16_t crc16 = Get_Crc16((uint8_t *)&status_buff[2], 54);
					memcpy((uint8_t *)&bat_status_message, (uint8_t *)&status_buff[0], 56);
					if(crc16 == bat_status_message.crc16) 
					{	//校验通过
						float stVolt = bat_status_message.bat_series_num*3.8f;
						float powerUsage = stVolt * (bat_basics_message.batteryCapacityExpected - bat_detect_message.batteryCapacityRemain)*0.1f;
						batteryUpdate( 0, driver_info.sensor_key,
										true,	//available
										bat_status_message.bus_voltage*0.01f,	//totalVoltRaw
										bat_status_message.bus_voltage*0.01f, //totalVolt
										stVolt,	//stVolt
										bat_status_message.bus_current*-0.001f,	//total current
										&powerUsage,	//power usage
										stVolt * bat_basics_message.batteryCapacityExpected*0.11f, //capacity
										bat_status_message.bat_capacity*0.1f,	//percent
										bat_status_message.bat_temp*0.1f,	//temperature
										bat_detect_message.useCircleCount, //cycle count
										bat_status_message.bat_status,	//error flags
										0, 0 );
					}
					break;
				}
			}
		}
	}
}

static bool CAN_Radar_BMS_ZHIAN_DriverInit()
{
	return true;
}
static bool CAN_Radar_BMS_ZHIAN_DriverRun()
{
	//注册电池
	uint32_t batId = batteryRegister(0);
	if( batId == 0 )
		return false;
	
	CanId Ids[11];
	//BMS发送实时信息包																														
	Ids[0].Identifier = ID_BATSTATUS(1,0,1);  Ids[0].IdType = 1;
	Ids[1].Identifier = ID_BATSTATUS(0,0,2);  Ids[1].IdType = 1;
	Ids[2].Identifier = ID_BATSTATUS(0,0,3);  Ids[2].IdType = 1;
	Ids[3].Identifier = ID_BATSTATUS(0,0,4);  Ids[3].IdType = 1;
	Ids[4].Identifier = ID_BATSTATUS(0,0,5);  Ids[4].IdType = 1;
	Ids[5].Identifier = ID_BATSTATUS(0,0,6);  Ids[5].IdType = 1;
	Ids[6].Identifier = ID_BATSTATUS(0,1,7);  Ids[6].IdType = 1;
	//BMS基本信息
	Ids[7].Identifier = (0x0 << 26) | (MSG_BASICS << 20) | (0x7 << 17) | (MY_ID << 11) | (BAT_ID << 5) | (1 << 4) | (1 << 3) | (0 << 0);  Ids[7].IdType = 1;
	//BMS监测信息
	Ids[8].Identifier 	= ID_DETECT(1,0,1);  Ids[8].IdType = 1;
	Ids[9].Identifier 	= ID_DETECT(0,0,2);  Ids[9].IdType = 1;
	Ids[10].Identifier 	= ID_DETECT(0,1,3);  Ids[10].IdType = 1;
	
	CanMailBox* mail_box = new CanMailBox( 5, Ids, 11 );
	DriverInfo* driver_info = new DriverInfo;
	driver_info->mail_box = mail_box;
	driver_info->sensor_key = batId;
	xTaskCreate( CAN_Radar_BMS_ZHIAN_Server, "CAN_Radar_BMS_ZHIAN", 800, (void*)driver_info, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_CAN_BMS_ZHIAN()
{
	CanFunc_Register( 8, CAN_Radar_BMS_ZHIAN_DriverInit, CAN_Radar_BMS_ZHIAN_DriverRun );
}