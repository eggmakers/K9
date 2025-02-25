#include "drv_CAN_Engine_Zongshen.hpp"
#include "drv_CAN.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"
#include "MavlinkCMDProcess.hpp"

static uint8_t componentID = MAV_COMP_ID_ENGINE_ZONGSHEN;

struct DriverInfo
{
	CanMailBox* mail_box;
	uint32_t sensor_key;
};


static void send_msg_to_gcs(uint8_t msg_type, uint8_t target_network, uint8_t* payload)
{
	for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
	{	
		const Port* port = get_CommuPort(i);
		if( port->write != 0 )
		{	
			if( mavlink_lock_chan(i,0.01) )
			{		
				mavlink_message_t msg_sd;
				mavlink_msg_component_extension43_pack_chan(
					get_CommulinkSysId() , //system id
					get_CommulinkCompId() ,//component id
					i ,										 //chan
					&msg_sd,
					target_network,				 //target_network
					get_CommulinkSysId(),	 //target system
					componentID,					 //target component
					msg_type,							 //message_type
					payload);					
				mavlink_msg_to_send_buffer( port->write, 
															port->lock,
															port->unlock,
															&msg_sd, 0.01, 0.01);		
				mavlink_unlock_chan(i);
			}	
		}
	}

}


	
static void CAN_Engine_Zongshen_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	struct engine_status
	{
		uint8_t firmwareVersion;
		uint8_t name[3];
		uint8_t productionYear;
		uint8_t productionMonth;
		uint8_t productionDay;
		uint8_t productionSeq;
		
		//发动机工作状态
		uint16_t engineStatus;
		//发动机转速
		uint16_t motorRpm;
		//输出电压
		uint16_t voltageOutput;
		//输出电流
		uint16_t currentOutput;
		//缸体温度
		uint16_t cylinderTemp;
		//冷却液温度
		uint16_t coolantTemperature;
		//节气门位置
		uint16_t throttlePosition;
		//油位
		uint16_t gasolineLevel;
		//总运行时间
		uint32_t systemRumTime;
		//剩余保养时间
		uint16_t timeForService;
		//剩余锁机时间
		uint16_t timeForLocking;
		//故障 
		uint8_t faultFlag;
		//报警1
		uint8_t alarmFlag1;
		//报警2
		uint8_t alarmFlag2;		
	}__PACKED;
	
	engine_status _engine_status;
	memset(&_engine_status,0,sizeof(_engine_status));
	
	TIME msg_send_time;
	while(1)
	{
		CmdMsg msg;
		bool msgAvailable=ReceiveCmdMsgFromTask( &msg, componentID, 0.005);
		if(msgAvailable)
		{
			if(msg.cmd == MAV_CMD_DO_ENGINE_CONTROL)
			{
				if(msg.params[0] == 0){// 启动发动机
					CanPacket mail;
					mail.IdType = 0;
					mail.DataLength = 8;
					mail.FrameType = 0;
					mail.FDFormat = 0;
					mail.Identifier = 0x1A0;
					mail.data[0] = 1; // 0 – stop ，1 – start 
					mail.data[1] = 0;
					mail.data[2] = 0;
					mail.data[3] = 0;
					mail.data[4] = 0;
					mail.data[5] = 0;
					mail.data[6] = 0;
					mail.data[7] = 1; // 校验和
					driver_info.mail_box->SendMail(mail, 0.2, 0.2);
					//sendLedSignal(LEDSignal_Err1);
				}else if(msg.params[0] == 1){// 关闭发动机
					CanPacket mail;
					mail.IdType = 0;
					mail.DataLength = 8;
					mail.FrameType = 0;
					mail.FDFormat = 0;
					mail.Identifier = 0x1A0;
					mail.data[0] = 0; // 0 – stop ，1 – start 
					mail.data[1] = 0;
					mail.data[2] = 0;
					mail.data[3] = 0;
					mail.data[4] = 0;
					mail.data[5] = 0;
					mail.data[6] = 0;
					mail.data[7] = 1; // 校验和
					driver_info.mail_box->SendMail(mail, 0.2, 0.2);
					//sendLedSignal(LEDSignal_Success1);
				}
			}
		}	
		
		CanPacket mail;
		if( driver_info.mail_box->receiveMail( &mail, 0.04 ) )
		{
			if( mail.Identifier == 0x1C0 )
			{	//目标信息包
				struct target_info_packet
				{
					uint8_t firmwareVersion;
					uint8_t name[3];
					uint8_t productionYear;
					uint8_t productionMonth;
					uint8_t productionDay;
					uint8_t productionSeq;
				}__PACKED;
				target_info_packet* packet = (target_info_packet*)&mail.data;
				uint8_t payload[43];
				payload[0] = packet->firmwareVersion;
				memcpy(&payload[1],packet->name,3);
				payload[4] = packet->productionYear;
				payload[5] = packet->productionMonth;
				payload[6] = packet->productionDay;
				payload[7] = packet->productionSeq;
				
				_engine_status.firmwareVersion = packet->firmwareVersion;
				memcpy(&_engine_status.name[0],packet->name,3);
				_engine_status.productionYear 	= packet->productionYear;
				_engine_status.productionMonth  = packet->productionMonth;
				_engine_status.productionDay 	  = packet->productionDay;
				_engine_status.productionSeq 	  = packet->productionSeq;
				
				send_msg_to_gcs(0x01, packet->productionSeq, payload);
			}
			else if( mail.Identifier == 0x1C1 )
			{	//数据包
				struct target_data_packet
				{
					uint8_t rpm;
					uint8_t throttlePosition;
					uint8_t oilLevel;
					uint8_t cylinderTemp;
					uint8_t coolantTemp;
					uint8_t engineState;
				}__PACKED;
				target_data_packet* packet = (target_data_packet*)&mail.data;
				_engine_status.motorRpm 			    = packet->rpm;
				_engine_status.throttlePosition   = packet->throttlePosition;
				_engine_status.gasolineLevel 		  = packet->oilLevel;
				_engine_status.cylinderTemp 		  = packet->cylinderTemp;
				_engine_status.coolantTemperature = packet->coolantTemp;
				_engine_status.motorRpm 				  = packet->rpm;
				_engine_status.engineStatus 			= packet->engineState;
			}
			else if( mail.Identifier == 0x1C2 )
			{	//数据包
				struct target_data_packet
				{
					uint16_t outputVoltage;
					uint16_t outputCurrent;
					uint16_t alarmFlag;
					uint8_t rsv;
					uint8_t faultFlag;
				}__PACKED;
				target_data_packet* packet = (target_data_packet*)&mail.data;
				_engine_status.voltageOutput 	= packet->outputVoltage;
				_engine_status.currentOutput  = packet->outputCurrent;
				_engine_status.alarmFlag1 	  = ((uint8_t*)&packet->alarmFlag)[0];
				_engine_status.alarmFlag2 		= ((uint8_t*)&packet->alarmFlag)[1];
				_engine_status.faultFlag 		  = packet->faultFlag;
			}			
			else if( mail.Identifier == 0x1C4 )
			{	//数据包
				struct target_data_packet
				{
					uint32_t totalRunTime;
					uint16_t timeForService;
					uint16_t timeForLocking;
				}__PACKED;
				target_data_packet* packet = (target_data_packet*)&mail.data;
				_engine_status.systemRumTime 			= packet->totalRunTime;
				_engine_status.timeForService     = packet->timeForService;
				_engine_status.timeForLocking 		= packet->timeForLocking;
			}		
			
			
			
			if(msg_send_time.get_pass_time()>0.5 && 
				_engine_status.productionYear!=0 && 
				_engine_status.productionMonth!=0 && 
			  _engine_status.productionDay!=0 
			){
				msg_send_time = TIME::now();
				uint8_t payload[43];
				payload[0] = _engine_status.engineStatus;
				memcpy(&payload[1],&_engine_status.motorRpm,2);
			  memcpy(&payload[3],&_engine_status.voltageOutput,2);
				memcpy(&payload[5],&_engine_status.currentOutput,2);
				memcpy(&payload[9],&_engine_status.gasolineLevel,2);
			  memcpy(&payload[11],&_engine_status.cylinderTemp,1);
				int invalid = -200;
				payload[12] = *(uint8_t*)&invalid; // 无效不显示
		    memcpy(&payload[13],&_engine_status.coolantTemperature,1);
			  payload[14] = *(uint8_t*)&invalid; // 无效不显示
				payload[15] = *(uint8_t*)&invalid; // 无效不显示
				payload[16] = *(uint8_t*)&invalid; // 无效不显示
				payload[17] = *(uint8_t*)&invalid; // 无效不显示
				payload[18] = *(uint8_t*)&invalid; // 无效不显示
				payload[19] = *(uint8_t*)&invalid; // 无效不显示
				memcpy(&payload[20],&_engine_status.throttlePosition,2);
				memcpy(&payload[22],&_engine_status.systemRumTime,4);
				memcpy(&payload[26],&_engine_status.timeForService,2);
				memcpy(&payload[28],&_engine_status.timeForLocking,2);
				memcpy(&payload[30],&_engine_status.faultFlag,1);
				memcpy(&payload[31],&_engine_status.alarmFlag1,1);
				memcpy(&payload[32],&_engine_status.alarmFlag2,1);		
				send_msg_to_gcs(0x02, _engine_status.productionSeq, payload);
			}
		}
		
//		//测试
//		if(msg_send_time.get_pass_time()>0.5){
//				msg_send_time = TIME::now();
//				uint8_t payload[43];
//				payload[0] = 1;
//				char name[4]; 
//				strcpy(name, "GAE");
//				memcpy(&payload[1],name,3);
//				payload[4] = 20;
//				payload[5] = 5;
//				payload[6] = 14;
//				payload[7] = 19;
//				send_msg_to_gcs(0x01, 1, payload);
//			
//				_engine_status.engineStatus = 1;
//				_engine_status.voltageOutput++;
//			  _engine_status.currentOutput++;
//			  _engine_status.gasolineLevel++;
//				_engine_status.cylinderTemp++;
//				_engine_status.coolantTemperature = 94;
//				_engine_status.throttlePosition = 5;
//				_engine_status.systemRumTime = 60*60;
//			  _engine_status.timeForService = 300;
//				_engine_status.timeForLocking = 10;
//				_engine_status.faultFlag = 1;
//				_engine_status.alarmFlag1 = 0x0f;
//				_engine_status.alarmFlag2 = 0x0f;
//				
//				memcpy(&payload[1],&_engine_status.motorRpm,2);
//			  memcpy(&payload[3],&_engine_status.voltageOutput,2);
//				memcpy(&payload[5],&_engine_status.currentOutput,2);
//				memcpy(&payload[9],&_engine_status.gasolineLevel,2);
//			  memcpy(&payload[11],&_engine_status.cylinderTemp,1);
//				int8_t invalid = -127;
//				payload[12] = *(uint8_t*)&invalid; // 无效不显示
//		    memcpy(&payload[13],&_engine_status.coolantTemperature,1);
//			  payload[14] = *(uint8_t*)&invalid; // 无效不显示
//				payload[15] = *(uint8_t*)&invalid; // 无效不显示
//				payload[16] = *(uint8_t*)&invalid; // 无效不显示
//				payload[17] = *(uint8_t*)&invalid; // 无效不显示
//				payload[18] = *(uint8_t*)&invalid; // 无效不显示
//				payload[19] = *(uint8_t*)&invalid; // 无效不显示
//				memcpy(&payload[20],&_engine_status.throttlePosition,2);
//				memcpy(&payload[22],&_engine_status.systemRumTime,4);
//				memcpy(&payload[26],&_engine_status.timeForService,2);
//				memcpy(&payload[28],&_engine_status.timeForLocking,2);
//				memcpy(&payload[30],&_engine_status.faultFlag,1);
//				memcpy(&payload[31],&_engine_status.alarmFlag1,1);
//				memcpy(&payload[32],&_engine_status.alarmFlag2,1);		
//				send_msg_to_gcs(0x02, 1, payload);
//			}
		
	}
}



static bool CAN_Engine_Zongshen_DriverInit()
{
	return true;
}
static bool CAN_Engine_Zongshen_DriverRun()
{
	// 注册消息队列
	TaskQueueRegister(componentID,30);
	
	CanId Ids[5];
	Ids[0].Identifier = 0x1C0;	Ids[0].IdType = 0;
	Ids[1].Identifier = 0x1C1;	Ids[1].IdType = 0;
	Ids[2].Identifier = 0x1C2;	Ids[2].IdType = 0;
	Ids[3].Identifier = 0x1C3;	Ids[3].IdType = 0;
	Ids[4].Identifier = 0x1C4;	Ids[4].IdType = 0;
	CanMailBox* mail_box = new CanMailBox( 10, Ids, 5 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->mail_box = mail_box;
	xTaskCreate( CAN_Engine_Zongshen_Server, "CAN_Engine_Zongshen", 1024, (void*)driver_info, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_Engine_Zongshen()
{
	CanFunc_Register( 96, CAN_Engine_Zongshen_DriverInit, CAN_Engine_Zongshen_DriverRun );
}