#include "drv_CAN_Engine_Xuanfu_16KW.hpp"
#include "drv_CAN.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"
#include "MavlinkCMDProcess.hpp"
#include "StorageSystem.hpp"

static uint8_t componentID = MAV_COMP_ID_ENGINE_XUANFU_16KW;

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
void send_mail(DriverInfo* driver_info, CanPacket mail, double timeout1, double timeout2 )
{
	if(driver_info){
		driver_info->mail_box->SendMail(mail, 0.2, 0.2);
		send_ind++;
		if(send_ind==16)
			send_ind=0;
	}
}

extern float debug_test[30];
static void CAN_Engine_Xuanfu_Server(void* pvParameters)
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
		uint16_t gasolineLevel;
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
		
		uint8_t EmgST0; // 超限故障0
		uint8_t EmgST1; // 超限故障1
		uint8_t ErrST0; // 一般故障状态0 
		uint8_t ErrST1; // 一般故障状态1 
		uint8_t ErrST2; // 一般故障状态2 
		uint8_t ErrST3; // 一般故障状态3 	
		uint8_t AlmST0; // 异常报警0
		uint8_t AlmST1; // 异常报警1
		uint8_t AlmST2; // 异常报警2 
		uint8_t AlmST3; // 异常报警3 
		uint8_t SysST0; // 系统状态0 
		uint8_t SysST1; // 系统状态1 
		uint8_t SysST2; // 系统状态2 
		uint8_t SysST3; // 系统状态3
	}__PACKED;
	
	engine_status _engine_status;
	memset(&_engine_status,0,sizeof(_engine_status));
	
	TIME msg_send_time;
	TIME engine_msg_request_send_time;
	TIME engine_heartBeat_send_time;
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
					
					CanPacket mail;
					mail.IdType = 0;
					mail.DataLength = 8;
					mail.FrameType = 0;
					mail.FDFormat = 0;
					mail.Identifier = 0x1E0;
					mail.data[0] = 0x12; 
					mail.data[1] = 0x80; 
					mail.data[2] = 0;
					mail.data[3] = 0;
					mail.data[4] = 0;
					mail.data[5] = 0;
					mail.data[6] = (send_ind<<4);
					mail.data[7] = (uint8_t)(0x80+(send_ind<<4)); 
					mail.data[7] = 0-mail.data[7];
					//send_mail(&driver_info, mail, 0.2, 0.2);	
					//sendLedSignal(LEDSignal_Err1);
				}else if(msg.params[0] == 0){// 关闭发动机
					engine_start_ctrl = false;
					
					CanPacket mail;
					mail.IdType = 0;
					mail.DataLength = 8;
					mail.FrameType = 0;
					mail.FDFormat = 0;
					mail.Identifier = 0x1E0;
					mail.data[0] = 0x12; 
					mail.data[1] = 0; // 0 – stop ，1 – start 
					mail.data[2] = 0;
					mail.data[3] = 0;
					mail.data[4] = 0;
					mail.data[5] = 0;
					mail.data[6] = (send_ind<<4);
					mail.data[7] = (uint8_t)(0x12+(send_ind<<4)); // 校验和
					mail.data[7] = 0-mail.data[7];
					//send_mail(&driver_info, mail, 0.2, 0.2);	
					//sendLedSignal(LEDSignal_Success1);
				}
			}
		}
		
		if(engine_msg_request_send_time.get_pass_time()>0.02)
		{
			engine_msg_request_send_time = TIME::now();
			if(_engine_status.productionYear	==0  && 
				 _engine_status.productionMonth	==0  && 
				 _engine_status.productionDay	  ==0 )
			{
				// 读软件运行时间
				CanPacket mail;
				mail.IdType = 0;
				mail.DataLength = 8;
				mail.FrameType = 0;
				mail.FDFormat = 0;
				mail.Identifier = 0x1E0;
				mail.data[0] = 0x16; 
				mail.data[1] = 0; 
				mail.data[2] = 0;
				mail.data[3] = 0;
				mail.data[4] = 0;
				mail.data[5] = 0;
				mail.data[6] = (send_ind<<4);
				mail.data[7] = (uint8_t)(0x16+(send_ind<<4)); // 校验和
				mail.data[7] = (uint8_t)(0-mail.data[7]);
				send_mail(&driver_info, mail, 0.2, 0.2);					
			}
			else
			{
				CanPacket mail;
				mail.IdType = 0;
				mail.DataLength = 8;
				mail.FrameType = 0;
				mail.FDFormat = 0;
				mail.Identifier = 0x1E0;
				mail.data[0] = 0x12; 
				mail.data[1] = engine_start_ctrl ? 0x80 : 0; 
				mail.data[2] = 0;
				mail.data[3] = 0;
				mail.data[4] = 0;
				mail.data[5] = 0;
				mail.data[6] = (send_ind<<4);
				mail.data[7] = (uint8_t)(mail.data[0]+mail.data[1]+mail.data[6]); // 校验和
				mail.data[7] = (uint8_t)(0-mail.data[7]);
				send_mail(&driver_info, mail, 0.2, 0.2);	


				if(engine_heartBeat_send_time.get_pass_time()>5)
				{
					engine_heartBeat_send_time = TIME::now();
					uint8_t payload[43];
					payload[0] = _engine_status.firmwareVersion;
					memcpy(&payload[1],_engine_status.name,3);
					payload[4] = _engine_status.productionYear;
					payload[5] = _engine_status.productionMonth;
					payload[6] = _engine_status.productionDay;
					payload[7] = _engine_status.productionSeq;
					send_msg_to_gcs(0x01, _engine_status.productionSeq, payload);
					
					// 发动机运行时间
					CanPacket mail;
					mail.IdType = 0;
					mail.DataLength = 8;
					mail.FrameType = 0;
					mail.FDFormat = 0;
					mail.Identifier = 0x1E0;
					mail.data[0] = 0x17; 
					mail.data[1] = 0; 
					mail.data[2] = 0;
					mail.data[3] = 0;
					mail.data[4] = 0;
					mail.data[5] = 0;
					mail.data[6] = (send_ind<<4);
					mail.data[7] = (uint8_t)(0x17+(send_ind<<4));
					mail.data[7] = (uint8_t)(0-mail.data[7]);
					send_mail(&driver_info, mail, 0.2, 0.2);
				}
			}					
		}
				
		
		CanPacket mail;
		if( driver_info.mail_box->receiveMail( &mail, 0.005 ) )
		{
			//debug_test[21] ++;
			//debug_test[23] = mail.Identifier;
			if( mail.Identifier == 0x1E2 )
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
			else if( mail.Identifier == 0x1E3 )
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
			else if( mail.Identifier == 0x1E8 )
			{	//数据包
				struct target_data_packet
				{
					uint32_t timeStamp;
					uint8_t engineState;
					uint8_t coolantTemp1;
					uint8_t coolantTemp2;
					uint8_t cylinderTemp;
				}__PACKED;
				target_data_packet* packet = (target_data_packet*)&mail.data;
				_engine_status.cylinderTemp 		  = packet->cylinderTemp - 40;
				_engine_status.coolantTemperature = packet->coolantTemp1 - 40;
				_engine_status.coolantTemperature2 = packet->coolantTemp2- 40;
				_engine_status.engineStatus 			= packet->engineState;
			}
			else if( mail.Identifier == 0x1E9 )
			{	//数据包
				struct target_data_packet
				{
					uint16_t engineSpeed;
					uint16_t outputVoltage;
					uint16_t outputCurrent;
					uint16_t batteryCurrent;
				}__PACKED;
				target_data_packet* packet = (target_data_packet*)&mail.data;
				_engine_status.motorRpm = packet->engineSpeed;
				_engine_status.voltageOutput 	= packet->outputVoltage*0.01;
				_engine_status.currentOutput  = packet->outputCurrent*0.05-400;
				_engine_status.batteryCurrent  = packet->batteryCurrent*0.05-400;
			}			
			else if( mail.Identifier == 0x1EA )
			{	//数据包
				struct target_data_packet
				{
					uint32_t TP1;
					uint16_t RSV;
					uint8_t BARO;
					uint8_t IAT;
				}__PACKED;
				target_data_packet* packet = (target_data_packet*)&mail.data;
				_engine_status.throttlePosition = (packet->TP1 & 0x00fff000>>12)*0.05;
				_engine_status.baro = packet->BARO;
				_engine_status.iat  = packet->IAT-40;
			}	
			else if( mail.Identifier == 0x1EB )
			{
				struct target_data_packet
				{
					uint16_t oilComsue;
					uint8_t gasLevel;
					uint8_t voltage12;
					uint8_t voltage5;
					uint8_t voltageServo;
					uint8_t voltageVBAT;
					uint8_t voltageVREF;
				}__PACKED;
				target_data_packet* packet = (target_data_packet*)&mail.data;
				_engine_status.gasolineLevel = (packet->gasLevel)*0.5;
			}	
			else if( mail.Identifier == 0x1EC )
			{	//数据包
				struct target_data_packet
				{
					uint8_t EmgST0; // 超限故障0
					uint8_t EmgST1; // 超限故障1
					uint8_t ErrST0; // 一般故障状态0 
					uint8_t ErrST1; // 一般故障状态1 
					uint8_t ErrST2; // 一般故障状态2 
					uint8_t ErrST3; // 一般故障状态3 
												
				}__PACKED;
				target_data_packet* packet = (target_data_packet*)&mail.data;
				_engine_status.EmgST0 = packet->EmgST0;
				_engine_status.EmgST1 = packet->EmgST1;
				_engine_status.ErrST0 = packet->ErrST0;
				_engine_status.ErrST1 = packet->ErrST1;
				_engine_status.ErrST2 = packet->ErrST2;
			  _engine_status.ErrST3 = packet->ErrST3;
			}							
			else if( mail.Identifier == 0x1ED )
			{	//数据包
				struct target_data_packet
				{
					uint8_t AlmST0; // 异常报警0
					uint8_t AlmST1; // 异常报警1
					uint8_t AlmST2; // 异常报警2 
					uint8_t AlmST3; // 异常报警3 
					uint8_t SysST0; // 系统状态0 
					uint8_t SysST1; // 系统状态1 
					uint8_t SysST2; // 系统状态2 
					uint8_t SysST3; // 系统状态3
				}__PACKED;
				target_data_packet* packet = (target_data_packet*)&mail.data;
				_engine_status.AlmST0 = packet->AlmST0;
				_engine_status.AlmST1 = packet->AlmST1;
				_engine_status.AlmST2 = packet->AlmST2;
				_engine_status.AlmST3 = packet->AlmST3;
				_engine_status.SysST0 = packet->SysST0;
				_engine_status.SysST1 = packet->SysST1;
				_engine_status.SysST2 = packet->SysST2;
				_engine_status.SysST3 = packet->SysST3;		
			}				
			
			
			if(msg_send_time.get_pass_time()>0.5 && 
				_engine_status.productionYear!=0   && 
				_engine_status.productionMonth!=0  && 
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
				
				int invalid = -127;
				payload[12] = *(uint8_t*)&invalid; // 无效不显示
		    memcpy(&payload[13],&_engine_status.coolantTemperature,1);
				memcpy(&payload[14],&_engine_status.coolantTemperature2,1);
				memcpy(&payload[15],&_engine_status.cylinderTemp,1);
			  memcpy(&payload[16],&_engine_status.iat,1);
			  memcpy(&payload[17],&_engine_status.baro,1);
				
				payload[18] = *(uint8_t*)&invalid; // 无效不显示
				payload[19] = *(uint8_t*)&invalid; // 无效不显示
				
				memcpy(&payload[20],&_engine_status.throttlePosition,2);
				memcpy(&payload[22],&_engine_status.systemRumTime,4);
				memcpy(&payload[26],&_engine_status.timeForService,2);
				memcpy(&payload[28],&_engine_status.timeForLocking,2);
				
				memcpy(&payload[30],&_engine_status.EmgST0,1);
				memcpy(&payload[31],&_engine_status.EmgST1,1);
				memcpy(&payload[32],&_engine_status.ErrST0,1);	
				memcpy(&payload[33],&_engine_status.ErrST1,1);	
				memcpy(&payload[34],&_engine_status.ErrST2,1);
				memcpy(&payload[35],&_engine_status.ErrST3,1);
				memcpy(&payload[36],&_engine_status.AlmST0,1);	
				memcpy(&payload[37],&_engine_status.AlmST1,1);	
				memcpy(&payload[38],&_engine_status.AlmST2,1);
				memcpy(&payload[39],&_engine_status.AlmST3,1);		
				
				static int cnt = 0;
				double buf[11];
				buf[0] = _engine_status.EmgST0;
				buf[1] = _engine_status.EmgST1;
				buf[2] = _engine_status.ErrST0;
				buf[3] = _engine_status.ErrST1;
				buf[4] = _engine_status.ErrST2;
				buf[5] = _engine_status.ErrST3;				
				buf[6] = _engine_status.AlmST0;
				buf[7] = _engine_status.AlmST1;
				buf[8] = _engine_status.AlmST2;
				buf[9] = _engine_status.AlmST3;
				buf[10] = cnt++;
				SDLog_Msg_DebugVect("Engine16KW",buf,11);
//				debug_test[13] = payload[36];
//				debug_test[14] = payload[37];
//				debug_test[15] = payload[38];
//				debug_test[16] = payload[39];
				
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
//			  _engine_status.coolantTemperature2 = 68;
//				_engine_status.throttlePosition = 5;
//				_engine_status.systemRumTime = 60*60;
//			  _engine_status.timeForService = 300;
//				_engine_status.timeForLocking = 10;

//				
//				memcpy(&payload[1],&_engine_status.motorRpm,2);
//			  memcpy(&payload[3],&_engine_status.voltageOutput,2);
//				memcpy(&payload[5],&_engine_status.currentOutput,2);
//				memcpy(&payload[9],&_engine_status.gasolineLevel,2);
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
//				
//				_engine_status.EmgST0 = 0;
//				_engine_status.EmgST1 = 0;
//				_engine_status.ErrST0 = 0;
//				_engine_status.ErrST1 = 0;
//				_engine_status.ErrST2 = 0;
//				_engine_status.ErrST3 = 0;
//				_engine_status.AlmST0 = 0;
//				_engine_status.AlmST1 = 18;
//				_engine_status.AlmST2 = 0;
//				_engine_status.AlmST3 = 0;
//				
//				memcpy(&payload[30],&_engine_status.EmgST0,1);
//				memcpy(&payload[31],&_engine_status.EmgST1,1);
//				memcpy(&payload[32],&_engine_status.ErrST0,1);	
//				memcpy(&payload[33],&_engine_status.ErrST1,1);	
//				memcpy(&payload[34],&_engine_status.ErrST2,1);
//				memcpy(&payload[35],&_engine_status.ErrST3,1);
//				memcpy(&payload[36],&_engine_status.AlmST0,1);	
//				memcpy(&payload[37],&_engine_status.AlmST1,1);	
//				memcpy(&payload[38],&_engine_status.AlmST2,1);
//				memcpy(&payload[39],&_engine_status.AlmST3,1);
//										
//				send_msg_to_gcs(0x02, 1, payload);
//			}
		
	}
}



static bool CAN_Engine_Xuanfu_DriverInit()
{
	return true;
}
static bool CAN_Engine_Xuanfu_DriverRun()
{
	// 注册消息队列
	TaskQueueRegister(componentID,30);
	
	CanId Ids[8];
	Ids[0].Identifier = 0x1E2;	Ids[0].IdType = 0;
	Ids[1].Identifier = 0x1E3;	Ids[1].IdType = 0;
	Ids[2].Identifier = 0x1E8;	Ids[2].IdType = 0;
	Ids[3].Identifier = 0x1E9;	Ids[3].IdType = 0;
	Ids[4].Identifier = 0x1EA;	Ids[4].IdType = 0;
	Ids[5].Identifier = 0x1EB;	Ids[5].IdType = 0;
	Ids[6].Identifier = 0x1EC;	Ids[6].IdType = 0;
	Ids[7].Identifier = 0x1ED;	Ids[7].IdType = 0;
	CanMailBox* mail_box = new CanMailBox( 100, Ids, 8 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->mail_box = mail_box;
	xTaskCreate( CAN_Engine_Xuanfu_Server, "CAN_Engine_Xuanfu", 2048, (void*)driver_info, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_Engine_Xuanfu_16KW()
{
	CanFunc_Register( 97, CAN_Engine_Xuanfu_DriverInit, CAN_Engine_Xuanfu_DriverRun );
}