#include "drv_CAN_Engine_Xuanfu_7KW.hpp"
#include "drv_CAN.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"
#include "MavlinkCMDProcess.hpp"
#include "ControlSystem.hpp"
#include "StorageSystem.hpp"
static uint8_t componentID = MAV_COMP_ID_ENGINE_XUANFU_7KW;

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

static int send_ind = 0;
static void send_mail(DriverInfo* driver_info, CanPacket mail, double timeout1, double timeout2 )
{
	if(driver_info){
		driver_info->mail_box->SendMail(mail, 0.2, 0.2);
		send_ind++;
		if(send_ind==16)
			send_ind=0;
	}
}

static uint8_t getSum(uint8_t* data, uint8_t len)
{
	uint8_t sum=0;
	for(int i=0; i<len; ++i){
		sum+=data[i];
	}
	return sum;
}

extern float debug_test[30];
static void CAN_Engine_Xuanfu_7KW_Server(void* pvParameters)
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
		uint16_t engineSpeed;
		//输出电压
		uint16_t voltageOutput;
		//输出电流
		int16_t currentOutput;
		//输出电流
		uint16_t batteryCurrent;
		//缸体温度
		uint16_t cylinderTemp;
		//冷却液温度
		int16_t coolantTemperature;
		//冷却液温度
		int16_t coolantTemperature2;
		//节气门位置
		uint16_t throttlePosition;
		//油位
		uint16_t fuelPosition;
		//总运行时间
		uint32_t systemRumTime;
		//剩余保养时间
		uint16_t timeForService;
		//剩余锁机时间
		uint16_t timeForLocking;
		//大气压力
		uint8_t baro;
		//进气温度
		int8_t iat;
		
		uint16_t extenderAlarm; 
		int8_t gearFault;

	}__PACKED;
	
	engine_status _engine_status;
	memset(&_engine_status,0,sizeof(_engine_status));
	
	TIME msg_send_time;
	TIME engine_msg_request_send_time;
	int engine_start_ctrl = false;
	while(1)
	{
		CmdMsg msg;
		bool msgAvailable=ReceiveCmdMsgFromTask( &msg, componentID, 0);
		if(msgAvailable)
		{
			if(msg.cmd == MAV_CMD_DO_ENGINE_CONTROL)
			{
				if(msg.params[0] == 1){// 启动发动机
					engine_start_ctrl = true;
				}else if(msg.params[0] == 0){// 关闭发动机
					engine_start_ctrl = false;
				}else if(msg.params[0] == 3){// 设置锁机时间
					
				}else if(msg.params[0] == 2){// 设置维护时间
					
				}
			}
		}
		
		if(engine_msg_request_send_time.get_pass_time()>0.02)
		{
			engine_msg_request_send_time = TIME::now();
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			
			CanPacket mail;
			mail.IdType = 0;
			mail.DataLength = 8;
			mail.FrameType = 0;
			mail.FDFormat = 0;
			mail.Identifier = 0x1A0;
			mail.data[0] = engine_start_ctrl; 
			mail.data[1] = 0x10; 
			mail.data[2] = 0; // SettingParameter
			mail.data[3] = 0; // SettingParameter
			mail.data[4] = 0; // RSV
			mail.data[5] = 0; // RSV
			mail.data[6] = (send_ind<<4) | (inFlight ? 1 : 0);
			mail.data[7] = getSum(mail.data,7);// 校验和
			mail.data[7] = (uint8_t)(0-mail.data[7]);
			send_mail(&driver_info, mail, 0.2, 0.2);					
		}
				
		
		CanPacket mail;
		if( driver_info.mail_box->receiveMail( &mail, 0.005 ) )
		{
			//debug_test[21] ++;
			//debug_test[23] = mail.Identifier;
//			double buf[5];
//			buf[0] = mail.Identifier;
//			buf[1] = _engine_status.productionYear;
//			buf[2] = _engine_status.productionMonth;
//			buf[3] = _engine_status.productionDay;
//			SDLog_Msg_DebugVect("Engine7KW",buf,4);
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
				
				//debug_test[20] = _engine_status.productionYear;
				//debug_test[21] = _engine_status.productionMonth;
				//debug_test[22] ++;
			}
			else if( mail.Identifier == 0x1C1 )
			{	//数据包
				struct target_data_packet
				{
					uint16_t engineSpeed;
					uint16_t throttlePosition;
					uint8_t fuelPosition;
					uint8_t cylinderTemp;
					uint8_t coolantTemp;
					uint8_t engineState;
				}__PACKED;
				target_data_packet* packet = (target_data_packet*)&mail.data;
				_engine_status.engineSpeed 			  = packet->engineSpeed;
				_engine_status.throttlePosition 	= packet->throttlePosition * 0.1;
				_engine_status.fuelPosition 		  = packet->fuelPosition * 0.5;
				_engine_status.cylinderTemp 		  = packet->cylinderTemp - 40;
				_engine_status.coolantTemperature = packet->coolantTemp - 40;
				_engine_status.engineStatus 			= packet->engineState;
			}
			else if( mail.Identifier == 0x1C2 )
			{	//数据包
				struct target_data_packet
				{
					uint16_t outputVoltage;
					uint16_t outputCurrent;
					uint16_t extenderAlarm;
					uint8_t rsv;
					uint8_t gearFault;
				}__PACKED;
				target_data_packet* packet = (target_data_packet*)&mail.data;
				_engine_status.voltageOutput 	= packet->outputVoltage*0.2;
				_engine_status.currentOutput  = (packet->outputCurrent-1000)*0.2;
				_engine_status.extenderAlarm  = packet->extenderAlarm;
				_engine_status.gearFault  		= packet->gearFault;
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
				_engine_status.productionYear!=0   && 
				_engine_status.productionMonth!=0  && 
			  _engine_status.productionDay!=0 
			){
				msg_send_time = TIME::now();
				uint8_t payload[43];
				payload[0] = _engine_status.engineStatus;
				memcpy(&payload[1],&_engine_status.engineSpeed,2);
			  memcpy(&payload[3],&_engine_status.voltageOutput,2);
				memcpy(&payload[5],&_engine_status.currentOutput,2);
				memcpy(&payload[9],&_engine_status.fuelPosition,2);
			  memcpy(&payload[11],&_engine_status.cylinderTemp,1);
				
				int invalid = -127;
				payload[12] = *(uint8_t*)&invalid; // 无效不显示
		    memcpy(&payload[13],&_engine_status.coolantTemperature,1);
				payload[14] = *(uint8_t*)&invalid; // 无效不显示
				memcpy(&payload[15],&_engine_status.cylinderTemp,1);
			  payload[16] = *(uint8_t*)&invalid; // 无效不显示
			  payload[17] = *(uint8_t*)&invalid; // 无效不显示
				payload[18] = *(uint8_t*)&invalid; // 无效不显示
				payload[19] = *(uint8_t*)&invalid; // 无效不显示
				
				memcpy(&payload[20],&_engine_status.throttlePosition,2);
				memcpy(&payload[22],&_engine_status.systemRumTime,4);
				memcpy(&payload[26],&_engine_status.timeForService,2);
				memcpy(&payload[28],&_engine_status.timeForLocking,2);
				
				uint8_t* p = (uint8_t*)&_engine_status.extenderAlarm;
				memcpy(&payload[30],&p[0],1);
				memcpy(&payload[31],&p[1],1);
				memcpy(&payload[32],&_engine_status.gearFault,1);		
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
//			  _engine_status.fuelPosition++;
//				_engine_status.cylinderTemp++;
//				_engine_status.coolantTemperature = 94;
//			  _engine_status.coolantTemperature2 = 68;
//				_engine_status.throttlePosition = 5;
//				_engine_status.systemRumTime = 60*60;
//			  _engine_status.timeForService = 300;
//				_engine_status.timeForLocking = 10;

//				
//				memcpy(&payload[1],&_engine_status.engineSpeed,2);
//			  memcpy(&payload[3],&_engine_status.voltageOutput,2);
//				memcpy(&payload[5],&_engine_status.currentOutput,2);
//				memcpy(&payload[9],&_engine_status.fuelPosition,2);
//			  memcpy(&payload[11],&_engine_status.cylinderTemp,1);
//				int8_t invalid = -127;
//				payload[12] = *(uint8_t*)&invalid; // 无效不显示
//		    memcpy(&payload[13],&_engine_status.coolantTemperature,1);
//				memcpy(&payload[14],&_engine_status.coolantTemperature2,1);
//				memcpy(&payload[15],&_engine_status.cylinderTemp,1);
//			  memcpy(&payload[16],&_engine_status.iat,1);
//			  memcpy(&payload[17],&_engine_status.baro,1);
//				

//				payload[18] = *(uint8_t*)&invalid; // 无效不显示
//				payload[19] = *(uint8_t*)&invalid; // 无效不显示
//				
//				memcpy(&payload[20],&_engine_status.throttlePosition,2);
//				
//				memcpy(&payload[22],&_engine_status.systemRumTime,4);
//				memcpy(&payload[26],&_engine_status.timeForService,2);
//				memcpy(&payload[28],&_engine_status.timeForLocking,2);
//				uint8_t* p = (uint8_t*)&_engine_status.extenderAlarm;
//				memcpy(&payload[30],&p[0],1);
//				memcpy(&payload[31],&p[1],1);
//				memcpy(&payload[32],&_engine_status.gearFault,1);	
//				send_msg_to_gcs(0x02, 1, payload);
//			}
		
	}
}



static bool CAN_Engine_Xuanfu_7KW_DriverInit()
{
	return true;
}
static bool CAN_Engine_Xuanfu_7KW_DriverRun()
{
	// 注册消息队列
	TaskQueueRegister(componentID,30);
	
	CanId Ids[4];
	Ids[0].Identifier = 0x1C0;	Ids[0].IdType = 0;
	Ids[1].Identifier = 0x1C1;	Ids[1].IdType = 0;
	Ids[2].Identifier = 0x1C2;	Ids[2].IdType = 0;
	Ids[3].Identifier = 0x1C4;	Ids[3].IdType = 0;
	CanMailBox* mail_box = new CanMailBox( 100, Ids, 4 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->mail_box = mail_box;
	xTaskCreate( CAN_Engine_Xuanfu_7KW_Server, "CAN_Engine_Xuanfu_7KW", 2048, (void*)driver_info, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_Engine_Xuanfu_7KW()
{
	CanFunc_Register( 98, CAN_Engine_Xuanfu_7KW_DriverInit, CAN_Engine_Xuanfu_7KW_DriverRun );
}