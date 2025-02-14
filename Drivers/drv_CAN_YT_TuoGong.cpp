#include "drv_CAN_YT_TuoGong.hpp"
#include "drv_CAN.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"

static void CAN_YT_TuoGong_Server(void* pvParameters)
{
	#define freq 50
	CanMailBox* mail_box = (CanMailBox*)pvParameters;
	
	TIME pTIME, yTIME, zTIME, fTIME, sTIME, RtTIME;
	float last_fChann, last_sChann, last_RtChann;
	bool shooting_video = false;	uint8_t taking_photo = 255;	uint16_t photo_counter = 0;
	bool zooming = false;
	
	//初始化变量
	YTCtrl ty_ctrl;
	if( get_DgYTFocusCtrl(&ty_ctrl) )
		last_fChann = ty_ctrl.value;
	if( get_DgYTShootCtrl(&ty_ctrl) )
		last_sChann = ty_ctrl.value;
	if( get_DgYTReturnCtrl(&ty_ctrl) )
		last_RtChann = ty_ctrl.value;
	
	while(1)
	{
		os_delay(1.0/freq);
		
		bool RateCtrl = false;
		float yawRate = 0;	float pitRate = 0;
		if( get_DgYTPitCtrl(&ty_ctrl) )
		{	//俯仰控制
			if( ty_ctrl.update_TIME != pTIME )
			{
				pTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//角度控制
					CanPacket mail;
					mail.IdType = 1;
					mail.Identifier = 0xF01;
					mail.FDFormat = 0;
					mail.FrameType = 0;
					mail.DataLength = 8;
					mail.data[0] = 0x0f;
					mail.data[1] = 0x08;
					mail.data[2] = 0x02;	//1-航向 2-俯仰 3-倍数
					*(int16_t*)&mail.data[3] = (int16_t)(-ty_ctrl.value*10);
					mail_box->SendMail(mail);
				}
				else if( ty_ctrl.mode == 1 )
				{	//角速度控制
					RateCtrl = true;
					pitRate = -ty_ctrl.value;
				}
			}
		}
		if( get_DgYTYawCtrl(&ty_ctrl) )
		{	//偏航控制
			if( ty_ctrl.update_TIME != yTIME )
			{
				yTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//角度控制
					CanPacket mail;
					mail.IdType = 1;
					mail.Identifier = 0xF01;
					mail.FDFormat = 0;
					mail.FrameType = 0;
					mail.DataLength = 8;
					mail.data[0] = 0x0f;
					mail.data[1] = 0x08;
					mail.data[2] = 0x01;	//1-航向 2-俯仰 3-倍数
					*(int16_t*)&mail.data[3] = (int16_t)(-ty_ctrl.value*10);
					mail_box->SendMail(mail);
				}
				else if( ty_ctrl.mode == 1 )
				{	//角速度控制
					RateCtrl = true;
					yawRate = -ty_ctrl.value;
				}
			}
		}
		if( get_DgYTZoomCtrl(&ty_ctrl) )
		{	//倍数控制
			if( ty_ctrl.update_TIME != zTIME )
			{
				zTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//倍数控制
					CanPacket mail;
					mail.IdType = 1;
					mail.Identifier = 0xF01;
					mail.FDFormat = 0;
					mail.FrameType = 0;
					mail.DataLength = 8;
					mail.data[0] = 0x0f;
					mail.data[1] = 0x08;
					mail.data[2] = 0x03;	//1-航向 2-俯仰 3-倍数
					*(int16_t*)&mail.data[3] = (int16_t)(constrain(ty_ctrl.value,0.0f,200.0f)*100);
					mail_box->SendMail(mail);
				}
				else if( ty_ctrl.mode == 1 )
				{	//放大缩小控制
					uint8_t zoom = 0;
					if( ty_ctrl.value > 10 )
						zoom = 1;
					else if( ty_ctrl.value < -10 )
						zoom = 2;
					
					if( zoom )
					{
						CanPacket mail;
						mail.IdType = 1;
						mail.Identifier = 0xF01;
						mail.FDFormat = 0;
						mail.FrameType = 0;
						mail.DataLength = 8;
						mail.data[0] = 0x0f;
						mail.data[1] = 0x12;
						mail.data[2] = zoom;
						mail_box->SendMail(mail);
						zooming = true;
					}
					else if( zooming )
					{
						CanPacket mail;
						mail.IdType = 1;
						mail.Identifier = 0xF01;
						mail.FDFormat = 0;
						mail.FrameType = 0;
						mail.DataLength = 8;
						mail.data[0] = 0x0f;
						mail.data[1] = 0x12;
						mail.data[2] = 0x03;
						mail_box->SendMail(mail);
						zooming = false;
					}
				}
			}
		}
		if( get_DgYTFocusCtrl(&ty_ctrl) )
		{	//聚焦控制
			if( ty_ctrl.update_TIME != fTIME )
			{
				fTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//按钮按下聚焦
					if( ty_ctrl.value > 25 )
					{
						CanPacket mail;
						mail.IdType = 1;
						mail.Identifier = 0xF01;
						mail.FDFormat = 0;
						mail.FrameType = 0;
						mail.DataLength = 8;
						mail.data[0] = 0x0f;
						mail.data[1] = 0x13;
						mail.data[2] = 0x01;
						mail_box->SendMail(mail);
					}
				}
				else if( ty_ctrl.mode == 1 )
				{	//通道改变聚焦
					if( fabsf( ty_ctrl.value - last_fChann ) > 10 )
					{
						CanPacket mail;
						mail.IdType = 1;
						mail.Identifier = 0xF01;
						mail.FDFormat = 0;
						mail.FrameType = 0;
						mail.DataLength = 8;
						mail.data[0] = 0x0f;
						mail.data[1] = 0x13;
						mail.data[2] = 0x01;
						mail_box->SendMail(mail);
					}
					last_fChann = ty_ctrl.value;
				}
			}
		}
		if( get_DgYTShootCtrl(&ty_ctrl) )
		{	//拍照录像控制
			if( ty_ctrl.update_TIME != sTIME )
			{
				sTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//按钮上拍照 按钮下录像
					if( ty_ctrl.value > 25 )
					{
						if( taking_photo >= freq )
						{
							CanPacket mail;
							mail.IdType = 1;
							mail.Identifier = 0xF01;
							mail.FDFormat = 0;
							mail.FrameType = 0;
							mail.DataLength = 8;
							mail.data[0] = 0x0f;
							mail.data[1] = 0x10;
							mail.data[2] = 0x01;
							mail_box->SendMail(mail);
							taking_photo = 0;
						}
						else
							++taking_photo;
					}
					else if( ty_ctrl.value < -25 )
					{
						CanPacket mail;
						mail.IdType = 1;
						mail.Identifier = 0xF01;
						mail.FDFormat = 0;
						mail.FrameType = 0;
						mail.DataLength = 8;
						mail.data[0] = 0x0f;
						mail.data[1] = 0x11;
						mail.data[2] = 0x01;
						mail_box->SendMail(mail);
						shooting_video = true;
						taking_photo = 255;
					}
					else
					{
						if( shooting_video )
						{
							CanPacket mail;
							mail.IdType = 1;
							mail.Identifier = 0xF01;
							mail.FDFormat = 0;
							mail.FrameType = 0;
							mail.DataLength = 8;
							mail.data[0] = 0x0f;
							mail.data[1] = 0x11;
							mail.data[2] = 0x02;
							mail_box->SendMail(mail);
						}
						shooting_video = false;
						taking_photo = 255;
						photo_counter = 0;
					}
				}
				else if( ty_ctrl.mode == 1 )
				{	//通道改变拍照
					if( fabsf( ty_ctrl.value - last_sChann ) > 10 )
					{
						CanPacket mail;
						mail.IdType = 1;
						mail.Identifier = 0xF01;
						mail.FDFormat = 0;
						mail.FrameType = 0;
						mail.DataLength = 8;
						mail.data[0] = 0x0f;
						mail.data[1] = 0x10;
						mail.data[2] = 0x01;
						mail_box->SendMail(mail);
					}
					last_sChann = ty_ctrl.value;
				}
			}
		}
		if( get_DgYTReturnCtrl(&ty_ctrl) )
		{	//归位控制
			if( ty_ctrl.update_TIME != RtTIME )
			{
				RtTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//按钮按下归位
					if( ty_ctrl.value > 25 )
					{
						CanPacket mail;
						mail.IdType = 1;
						mail.Identifier = 0xF01;
						mail.FDFormat = 0;
						mail.FrameType = 0;
						mail.DataLength = 8;
						mail.data[0] = 0x0f;
						mail.data[1] = 0x03;
						mail_box->SendMail(mail);
						
					}
				}
				else if( ty_ctrl.mode == 1 )
				{	//通道改变归位
					if( fabsf( ty_ctrl.value - last_RtChann ) > 10 )
					{
						CanPacket mail;
						mail.IdType = 1;
						mail.Identifier = 0xF01;
						mail.FDFormat = 0;
						mail.FrameType = 0;
						mail.DataLength = 8;
						mail.data[0] = 0x0f;
						mail.data[1] = 0x03;
						mail_box->SendMail(mail);
					}
					last_RtChann = ty_ctrl.value;
				}
			}
		}
		
		if( RateCtrl )
		{	//控制角速度
			float yaw_rate=0, pit_rate=0;
			yaw_rate = remove_deadband( yawRate, 5.0f );
			pit_rate = remove_deadband( pitRate, 5.0f );
			if( yaw_rate!=0 || pit_rate!=0 )
			{
				CanPacket mail;
				mail.IdType = 1;
				mail.Identifier = 0xF01;
				mail.FDFormat = 0;
				mail.FrameType = 0;
				mail.DataLength = 8;
				mail.data[0] = 0x0f;
				mail.data[1] = 0x04;
				*(int16_t*)&mail.data[2] = (int16_t)(constrain(yaw_rate,-200.0f,200.0f)*100);
				*(int16_t*)&mail.data[4] = (int16_t)(constrain(pit_rate,-200.0f,200.0f)*100);
				mail_box->SendMail(mail);
			}
		}
	}
}



static bool CAN_YT_TuoGong_DriverInit()
{
	return true;
}
static bool CAN_YT_TuoGong_DriverRun()
{
	CanMailBox* mail_box = new CanMailBox( 0, 0, 2 );
	xTaskCreate( CAN_YT_TuoGong_Server, "CAN_YT_TuoGong", 800, (void*)mail_box, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_YT_TuoGong()
{
	CanFunc_Register( 32, CAN_YT_TuoGong_DriverInit, CAN_YT_TuoGong_DriverRun );
}