#include "drv_UM982_movingTrack.hpp"
#include "drv_RTK_DAO_Base.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "drv_CRC.hpp"
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include "precLand.hpp"
#include "ControlSystem.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
};

struct GpsConfig
{
	uint8_t GNSS_Mode[8];
	float delay[2];
};

enum GPS_Scan_Operation
{
	//��ָ���������·��ͳ�ʼ����Ϣ
	GPS_Scan_Baud9600 = 9600 ,
	GPS_Scan_Baud38400 = 38400 ,
	GPS_Scan_Baud460800 = 460800 ,
	GPS_Scan_Baud115200 = 115200 ,
	GPS_Scan_Baud57600 = 57600 ,
	
	//����Ƿ����óɹ�
	GPS_Check_Baud ,
	//�ڵ�ǰ���������ٴη�������
	GPS_ResendConfig ,
	//GPS�Ѽ�����
	GPS_Present ,
};

enum FrameName
{
	FrameNone=0,
	FrameKSXT,
	FrameGGA,
	FrameGSA,
	FrameGST,
	FrameGSV,
	FrameHDT,
	FrameHDT2,
	FrameGNRMC,
	FrameGPRMC,
	FrameGPVTG,
	FrameGPGLL,
	FrameGPZDA,
	FrameEVENTMARK
};

struct GPS_State_Machine
{
	uint16_t frame_id;
	uint8_t frameType;
	uint8_t header_length;
	FrameName frame_name = FrameNone;
	uint32_t frame_datas_ind = 0;
	uint16_t frame_datas_length;
	uint8_t read_state = 0;	
	uint16_t crc=0;
};

static inline void ResetRxStateMachine( GPS_State_Machine* state_machine )
{
	state_machine->read_state=state_machine->frame_datas_ind=0;
}

static inline uint16_t GPS_ParseByte( GPS_State_Machine* state_machine, uint8_t* frame_datas, uint8_t r_data )
{
	frame_datas[ state_machine->frame_datas_ind++ ] = r_data;
	switch( state_machine->read_state )
	{
		case 0:	//�Ұ�ͷ
		{				
			if( state_machine->frame_datas_ind == 1 )
			{
				if( r_data != 0xAA )
					state_machine->frame_datas_ind = 0;	
			}
			else if(state_machine->frame_datas_ind == 2)
			{
				if( r_data != 0x44 )
					state_machine->frame_datas_ind = 0;	
			}
			else
			{
				if( r_data == 0xB5 || r_data == 0x12)	//header found
				{
					if(r_data==0xB5)
						state_machine->header_length=24;
					else if(r_data==0x12)
						state_machine->header_length=28;
					state_machine->frameType=r_data;
					state_machine->read_state = 1;
//					state_machine->frame_datas_ind = 0;
					state_machine->crc = 0;	//reset checksum
				}		
				else
					state_machine->frame_datas_ind = 0;
			}	
			break;
		}
		case 1:	//��Class ID�Ͱ�����
		{
			if( state_machine->frameType == 0xB5 && state_machine->frame_datas_ind == 8 )
			{
				state_machine->frame_id = *(unsigned short*)&frame_datas[4];
				state_machine->frame_datas_length = (*(unsigned short*)&frame_datas[6]) + 24 + 4;
				if( state_machine->frame_datas_length > 28 && state_machine->frame_datas_length < 2000 )
					state_machine->read_state = 2;						
				else
					ResetRxStateMachine(state_machine);
			}
			else if( state_machine->frameType == 0x12 && state_machine->frame_datas_ind == 10 )
			{
				state_machine->frame_id = *(unsigned short*)&frame_datas[4];
				state_machine->frame_datas_length = (*(unsigned short*)&frame_datas[8]) + 28 + 4;
				if( state_machine->frame_datas_length > 32 && state_machine->frame_datas_length < 2000 )
					state_machine->read_state = 2;						
				else
					ResetRxStateMachine(state_machine);
			}
			break;
		}
		case 2:	//��������
		{
			if(state_machine->frameType == 0xB5)
			{
				if( state_machine->frame_datas_ind == state_machine->frame_datas_length - 12 )
				{
					//payload read completed
					state_machine->read_state = 3;
				}
			}
			else if(state_machine->frameType == 0x12)
			{
				if( state_machine->frame_datas_ind == state_machine->frame_datas_length - 14 )
				{
					//payload read completed
					state_machine->read_state = 3;
				}
			}
			break;
		}
		case 3://У��
		{
			if( state_machine->frame_datas_ind == state_machine->frame_datas_length)
			{
				uint32_t res =0;
				CRC_Calc( cg_CRC_CONFIG_CRC32_MAVLINK, (uint8_t*)&frame_datas[0], state_machine->frame_datas_length-4, &res);
				ResetRxStateMachine(state_machine);
				if( res == *(uint32_t*)&frame_datas[state_machine->frame_datas_length-4] )
					return state_machine->frame_datas_length;		
				else
					return 0;
			}
		}
	}
	return 0;
}

static bool send_init_msg(DriverInfo& driver_info)
{
  int len=0;
	char UM982Clr[100];
	len = sprintf( UM982Clr, "UNLOG COM2\r\n");	
	driver_info.port.write( (uint8_t*)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY );	
	driver_info.port.wait_sent(1);
	driver_info.port.reset_rx(0.1);
	len = sprintf( UM982Clr, "CONFIG\r\n");	
	driver_info.port.write( (uint8_t*)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY );	
	driver_info.port.wait_sent(1);
	os_delay(0.15);
	const char CmdResult[] = "$command,CONFIG,response: OK*54";
	const uint8_t CmdResultLen = strlen(CmdResult);
	len=0;
	TIME waitCorrectResponseTime = TIME::now();
	while( waitCorrectResponseTime.get_pass_time() < 3 )
	{
		if(driver_info.port.read( (uint8_t*)&UM982Clr[0], 1, 0.5, 1 )==0)
			return false;
		if( len < CmdResultLen )
		{
			if( UM982Clr[0] == CmdResult[len] )
				++len;
			else
				len = 0;
		}
		else
				break;
	}
	
	if( len != CmdResultLen )
		return false;

	// ������ƥ�����
	for( uint8_t i=0; i < 24; i++ )
	{
		if(i==0)      // ֹͣ������Ϣ���
			len = sprintf( UM982Clr,  "UNLOG COM2\r\n");	
			
		else if(i==1) // ����վ����ģʽ
			len = sprintf( UM982Clr,  "MODE HEADING2\r\n");	
		
		else if (i==2)// ���� RTK ģ��ָ�����������ڣ���Ϊ��λ
			len = sprintf( UM982Clr,  "CONFIG RTK TIMEOUT 600\r\n");
		
		else if (i==3)// ���ý��յ� DGPS������ݵ�������ڣ���Ϊ��λ�����յ����ͺ���ָ�����ڵ� DGPS ������ݱ����ԣ�Ҳ���ڽ�ֹ DGPS ��λ����
			len = sprintf( UM982Clr,  "CONFIG DGPS TIMEOUT 600\r\n");
		
		else if (i==4)// ����˫���߽��ջ��������ߣ�ANT1��������ߣ�ANT2��֮����뱣�̶ֹ�
			len = sprintf( UM982Clr,  "CONFIG HEADING FIXLENGTH\r\n");			
		
		else if (i==5)// ���� EVENT ���ܣ��½�����Ч��������Ч����֮������ʱ��Ҫ��, ��λms����С�� TGuard����ڶ��� Event �����ӡ�Ĭ��ֵ: 4����С�� 2�����3,599,999
			len = sprintf( UM982Clr,  "CONFIG EVENT ENABLE NEGATIVE 2\r\n");	
		
		else if (i==6)// ���ùرյ�Ƶ��λ״̬���ж����������߳��ȳ��� 5km ʱ����������������Ȼ����ʹ�ù̶���
			len = sprintf( UM982Clr,  "CONFIG SFRTK Enable\r\n");			
		
		else if (i==7)// �� BDS ϵͳ���ǵ� B1C&B2a �źű��� RTCM Э��
			len = sprintf( UM982Clr,  "CONFIG RTCMB1CB2a Enable\r\n");
		
		else if (i==8)// ʹ�ܽ��ջ����� GPS ����ϵ
			len = sprintf( UM982Clr,  "UNMASK GPS\r\n");		
		
		else if (i==9)// ʹ�ܽ��ջ����� BDS ����ϵͳ
			len = sprintf( UM982Clr,  "UNMASK BDS\r\n");
		
		else if (i==10)// ʹ�ܽ��ջ����� GLO ����ϵͳ
			len = sprintf( UM982Clr, "UNMASK GLO\r\n");
		
		else if (i==11)// ʹ�ܽ��ջ����� GAL ����ϵͳ
			len = sprintf( UM982Clr,  "UNMASK GAL\r\n");

		else if (i==12)// ʹ�ܽ��ջ����� QZSS ����ϵͳ
			len = sprintf( UM982Clr,  "UNMASK QZSS\r\n");
		
		else if (i==13)// ��� EVENT ����ʱ�̵ľ�ȷ����ʱ�估���ʱ��
			len = sprintf( UM982Clr,  "LOG EVENTMARKB ONCHANGED\r\n");	

	  else if (i==14)// ���ý��ջ��������ǵ����ǽ�ֹ�Ƕ�
			len = sprintf( UM982Clr,  "MASK 10\r\n");	
		
		else if (i==15)// ���ջ�Ĭ����Ϊ��̬ģʽ
			len = sprintf( UM982Clr,  "RTKDYNAMICS DYNAMIC\r\n");	
		
		else if (i==16)// ���ջ��˶��ĺ���
			len = sprintf( UM982Clr,  "UNIHEADINGB 0.1\r\n");

		else if (i==17)// UTC ʱ��
			len = sprintf( UM982Clr,  "LOG TIMEB ONTIME 0.1\r\n");

		else if (i==18)// ����������Ϣ
			len = sprintf( UM982Clr,  "STADOPB 1\r\n");
		
//		else if (i==19)// ԭʼ�۲�������Ϣ
//			len = sprintf( UM982Clr,  "LOG COM1 RANGEB ONTIME 0.1\r\n");		
//		
//		else if (i==20)// GLONASS ��������
//			len = sprintf( UM982Clr,  "LOG COM1 GLORAWSTRINGB ONCHANGED\r\n");				
	
//		else if (i==21)// GPS ��������
//			len = sprintf( UM982Clr,  "LOG COM1 RAWGPSSUBFRAMEB ONCHANGED\r\n");		
//		
//		else if (i==22)// BDS ��������
//			len = sprintf( UM982Clr,  "LOG COM1 BDSRAWNAVSUBFRAMEB ONCHANGED\r\n");	

//		else if (i==23)// QZSS ��������
//			len = sprintf( UM982Clr,  "LOG COM1 QZSSRAWSUBFRAMEB ONCHANGED\r\n");
		
		else if (i==19)// �����߼��������ѿ���λ�ú��ٶ�Bestnav
			len = sprintf( UM982Clr,  "BESTNAVB 0.05\r\n");

//		else if (i==22)// 20Hz �� GNGGA ��Ϣ
//			len = sprintf( UM982Clr,  "GPGGA 0.05\r\n");
				
		else if (i==20)// HEADING2
			len = sprintf( UM982Clr,  "LOG HEADING2B ONCHANGED\r\n");
		
		else if (i==21)// HEADING2
			len = sprintf( UM982Clr,  "HEADINGMODE VARIABLELENGTH\r\n");
		
		else if (i==22)// ����com2������Ϊ460800,8λ����λ,��У��,һλֹͣλ
			len = sprintf( UM982Clr,  "CONFIG COM1 460800 8 n 1\r\n");
		else if (i==23)// ����com2������Ϊ460800,8λ����λ,��У��,һλֹͣλ
			len = sprintf( UM982Clr,  "CONFIG COM2 460800 8 n 1\r\n");
	
		driver_info.port.write( (uint8_t*)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY );	
		driver_info.port.wait_sent(1);
		os_delay(0.1);
	}	
	return true;
}


static void RTK_Server(void* pvParameters)
{	
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	//ע�ᾫ׼���䴫����
	uint32_t precKey;
	do
	{
		precKey = precLandSensorRegister();
		os_delay(1.0);
	}while(precKey==0);
	
	//GPSʶ��״̬
	GPS_Scan_Operation current_GPS_Operation = GPS_Scan_Baud115200;
	//���ݶ�ȡ״̬��
	__attribute__((aligned(4))) uint8_t frame_datas[2048];
	//�ϴθ���ʱ��
	TIME last_update_time;
	
	//�ȴ���ʼ�����
	while( getInitializationCompleted() == false )
		os_delay(0.1);	
	
	//ע��Rtk�˿�
	RtkPort rtk_port;
	rtk_port.ena = false;
	rtk_port.write = driver_info.port.write;
	rtk_port.lock = driver_info.port.lock;
	rtk_port.unlock = driver_info.port.unlock;
	int8_t rtk_port_ind = RtkPortRegister(rtk_port);	
	
	bool rtc_updated = false;
	
	//��ȡ�Ƿ���Ҫ��¼PPK
	bool record_ppk = false;
	uint8_t log_ppk[8];
	if( ReadParam( "SDLog_PPK", 0, 0, (uint64_t*)log_ppk, 0 ) == PR_OK )
	{
		if( log_ppk[0] == 2 )
			record_ppk = true;
	}

GPS_CheckBaud:
	while(1)
	{
		//����ָ��������
		driver_info.port.SetBaudRate( current_GPS_Operation, 3, 0.1 );
		//�л�������
		switch(current_GPS_Operation)
		{
			case GPS_Scan_Baud9600:
				current_GPS_Operation = GPS_Scan_Baud57600;
				break;
			case GPS_Scan_Baud57600:
				current_GPS_Operation = GPS_Scan_Baud38400;
				break;
			case GPS_Scan_Baud38400:
				current_GPS_Operation = GPS_Scan_Baud460800;
				break;
			case GPS_Scan_Baud460800:
				current_GPS_Operation = GPS_Scan_Baud115200;
				break;
			default:
			case GPS_Scan_Baud115200:
				current_GPS_Operation = GPS_Scan_Baud9600;
				break;
		}
		//��������
    if( send_init_msg(driver_info) == false )
			continue;

		//���Ĳ�����
		driver_info.port.SetBaudRate( 460800, 1, 0.1 );		
		//��ս��ջ�����׼����������
		driver_info.port.reset_rx(0.1);
		GPS_State_Machine gps_state;
		ResetRxStateMachine(&gps_state);
		TIME RxChkStartTime = TIME::now();
		
		while( RxChkStartTime.get_pass_time()<3 )
		{
			uint8_t r_data;
			if( driver_info.port.read( &r_data, 1, 0.5, 0.1 ) )
			{
				bool res = GPS_ParseByte( &gps_state, frame_datas, r_data );
				if( res )
				{
					if( gps_state.frame_id == 2118 )
					{	//��ʶ��,��ת��gps���ճ���
						goto GPS_Present;
					}
				}
			}
		}		
	}	
	
GPS_Present:
	//�ط�����
	//send_init_msg(driver_info);
		
	uint32_t sensor_key = 0;
	uint32_t dao_key = 0;
	GpsDAOConfig gps_cfg;
	if( ReadParamGroup( "GPSDAOCfg", (uint64_t*)&gps_cfg, 0 ) == PR_OK )
	{	
		//ע�ᴫ����
		sensor_key = PositionSensorRegister( default_rtk_sensor_index , \
																					"RTK_UM982" ,\
																					Position_Sensor_Type_GlobalPositioning , \
																					Position_Sensor_DataType_sv_xy , \
																					Position_Sensor_frame_ENU , \
																					gps_cfg.delay[0] , //��ʱ
																					30 , //xy���ζ�
																					30 //z���ζ�
																				);	
		
		//ע����򴫸���
		dao_key = DAOSensorRegister( 0, "DRTK", vector3<double>(gps_cfg.DRTK_VecX[0],gps_cfg.DRTK_VecY[0],gps_cfg.DRTK_VecZ[0]), false, gps_cfg.delay[0] );
	}
	else
	{	//ע�ᴫ����
		sensor_key = PositionSensorRegister( default_rtk_sensor_index , \
																					"RTK_UM982" ,\
																					Position_Sensor_Type_GlobalPositioning , \
																					Position_Sensor_DataType_sv_xy , \
																					Position_Sensor_frame_ENU , \
																					0.1 , //��ʱ
																					30 , //xy���ζ�
																					30 //z���ζ�
																				);
		gps_cfg.DRTK_VecX[0] = gps_cfg.DRTK_VecY[0] = gps_cfg.DRTK_VecZ[0] = 0;
	}
	//����Rtkע��
	RtkPort_setEna( rtk_port_ind, true );
 
	//gps״̬
	bool gps_available = false;
	bool z_available=false;
	double alt_offset = -1000000;
	TIME GPS_stable_start_time(false);
	double gps_alt;
	TIME gps_update_TIME;
	
	//��ս��ջ�����׼����������
	driver_info.port.reset_rx(0.1);
	GPS_State_Machine gps_state,gps_state_nmea;
	ResetRxStateMachine(&gps_state);
	frame_datas[0] = frame_datas[1] = 0;
	last_update_time = TIME::now();
	
	struct Dop_Pack
	{	
		uint32_t itow;
		float gdop;//���ξ�������
		float pdop;//λ�þ�������
		float tdop;//ʱ�侫������
		float vdop;//��ֱ��������
		float hdop;//ˮƽ��������
		float ndop;//���򾫶�����
		float edop;//���򾫶�����
		float Cutoff;//�����߶Ƚ�  
		float rsv2;
		uint16_t svNum;
		uint16_t prn;
	}__attribute__((packed));
	Dop_Pack dop_pack = {0};		
	
	//��������
	double addition_inf[8] = {0};
	
	while(1)
	{
		uint8_t r_data;
		if( driver_info.port.read( &r_data, 1, 2, 0.1 ) )
		{
			if( GPS_ParseByte( &gps_state, frame_datas, r_data ) )
			{
				if( gps_state.frame_id == 2118 )
				{// BESTNAV ���λ�ú��ٶ�
					//��¼
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );		
					
					last_update_time = TIME::now();
					struct Best_Nav_Pack
					{	
						uint32_t PosStatus;//λ�ý�״̬
						uint32_t PosType;//λ������
						double lat;//γ�ȣ�deg
						double lon;//���ȣ�deg
						double height;//���θߣ�m
						float undulation;//���ˮ׼����- ���ˮ׼���WGS84 ������֮��ľ�,��
						uint32_t datum_id;//����ϵ ID ��,��ǰ��֧�� WGS84
						float LatAcc;//γ�ȱ�׼�m 
						float LonAcc;//���ȱ�׼�m 
						float HeightAcc;//�߶ȱ�׼�m 
						uint8_t stn_id[4];//��վ ID
						float diff_age;//������ڣ�s  
						float sol_age;//������ڣ�s  
						uint8_t SVTrack;//���ٵ�������  
						uint8_t SVUse;//�ڽ���ʹ�õ�������
						uint8_t rsv2[3];
						uint8_t ext_sol_stat;//��չ���״̬
						uint8_t mask1;//Galileo ʹ�õ��ź�����
						uint8_t mask2;//GPS, GLONASS �� BDS ʹ�õ��ź�����
						uint32_t VelStatus;//�ٶȽ�״̬
						uint32_t VelType;//�ٶ�����
						float latency;//���ٶ�ʱ�������ӳ�ֵ������Ϊ��λ
						float age;//������ڣ�s  
						double horSpeed;//�Ե�ˮƽ�ٶȣ�m/s 
						double horSpeedAngle;//������汱��ʵ�ʶԵ��˶�������Ե���켣����deg
						double VerticalSpeed;//��ֱ�ٶȣ�m/s
						float VerspdStd;//�߳��ٶȱ�׼���λ m/s
						float HorspdStd;//ˮƽ�ٶȱ�׼���λ m/s  
					}__attribute__((packed));
					Best_Nav_Pack* pack = (Best_Nav_Pack*)&frame_datas[gps_state.header_length];
				
					uint8_t gps_fix = 0;
					if(pack->PosType==16||(pack->PosType==17)||(pack->PosType==18))// ���㶨λ
						gps_fix = 3; 
					else if(pack->PosType==32||pack->PosType==33||pack->PosType==34)//RTK �����
						gps_fix = 5;	
					else if(pack->PosType==48||pack->PosType==49||pack->PosType==50)// RTK �̶���
						gps_fix = 6;
					
//					double hdop = (pack->LatAcc+pack->LonAcc)*0.5;
//					double vdop = pack->HeightAcc;
					double hdop = dop_pack.hdop;
					double vdop = dop_pack.vdop;	
					
					if( gps_fix!=0  && pack->SVTrack>= 7 )
					{
						if( gps_available == false )
						{
							if( hdop < 2.5 )
							{
								if( GPS_stable_start_time.is_valid() == false )
									GPS_stable_start_time = TIME::now();
								else if( GPS_stable_start_time.get_pass_time() > 3.0f )
								{
									gps_available = true;
									GPS_stable_start_time.set_invalid();
								}
							}
							else
								GPS_stable_start_time.set_invalid();
						}
						else
						{
							if( hdop > 3.5 )
								gps_available = z_available = false;
						}
					}
					else
					{
						gps_available = z_available = false;
						GPS_stable_start_time.set_invalid();
					}	
					
					addition_inf[0] = pack->SVTrack;
					addition_inf[1] = gps_fix;
					addition_inf[4] = hdop*100;
					addition_inf[5] = vdop*100;
					addition_inf[6] = 0;
					
					if( z_available )
					{
						if( vdop > 4.5 )
							z_available = false;
					}
					else
					{
						if( vdop < 2.5 )
						{
							gps_alt = pack->height * 1e2;
							z_available = true;
						}
					}
					double t = gps_update_TIME.get_pass_time_st();
					if( t > 1 )
						t = 1;	

					vector3<double> velocity;
					velocity.y = pack->horSpeed*cos(degree2rad(pack->horSpeedAngle))*100;	//North
					velocity.x = pack->horSpeed*sin(degree2rad(pack->horSpeedAngle))*100;	//East
					velocity.z = pack->VerticalSpeed*100;	//Up
					gps_alt += velocity.z*t;
					if( vdop < 6 )
					{	//�߾��ȸ߶�λ�ý��
						//ʹ�þ��Ը߶�
						double r_height = pack->height*100;
						if( alt_offset <= -100000 )
							alt_offset = r_height - gps_alt;
						gps_alt += 0.5*t * ( r_height - gps_alt - alt_offset );
					}
					else
						alt_offset = -1000000;
					
					vector3<double> position_Global;
					position_Global.x = pack->lat;
					position_Global.y = pack->lon;
					position_Global.z = gps_alt;

					if( z_available )
						PositionSensorChangeDataType( default_rtk_sensor_index,sensor_key, Position_Sensor_DataType_sv_xyz );
					else
						PositionSensorChangeDataType( default_rtk_sensor_index,sensor_key, Position_Sensor_DataType_sv_xy );
					
					//���ζ�
					double xy_trustD = hdop*100;
					double z_trustD = vdop*100;
					PositionSensorUpdatePositionGlobalVel( default_rtk_sensor_index,sensor_key, position_Global, velocity, 
						gps_available, //available
						-1, //delay
						xy_trustD, 
						z_trustD, 
						addition_inf,
						xy_trustD,	//xy LTtrust
						z_trustD	//z LTtrust
					);
				}
				else if( gps_state.frame_id == 972 )
				{// UNIHEADING ������Ϣ
					last_update_time = TIME::now();
					struct Heading_Pack
					{	
						uint32_t solStatus;//λ�ý�״̬
						uint32_t posType;//λ������
						float length;	//���߳� (0 �� 3000 m)  
						float heading;//���� (0 �� 360.0 deg)
						float pitch;	//����(��90 deg)  
						float rsv2;
						float hdgstddev;//�����׼ƫ��
						float ptchstddev;//������׼ƫ��
						uint8_t stn_id[4];//��վ ID
						uint8_t SVTrack;//���ٵ�������  
						uint8_t SVUse;//�ڽ���ʹ�õ�������
						uint8_t obs;//��ֹ�߶Ƚ����ϵ�������
						uint8_t multi;//��ֹ�߶Ƚ�������L2�۲��������
						uint8_t rsv3;
						uint8_t ext_sol_stat;//��չ���״̬
						uint8_t mask1;//Galileo ʹ�õ��ź�����
						uint8_t mask2;//GPS, GLONASS �� BDS ʹ�õ��ź�����						
					}__attribute__((packed));
					Heading_Pack* pack = (Heading_Pack*)&frame_datas[gps_state.header_length];			
					double len_pos = sq(gps_cfg.DRTK_VecX[0]) + sq(gps_cfg.DRTK_VecY[0]) + sq(gps_cfg.DRTK_VecZ[0]);
					bool available = false;
					if( len_pos > 7*7 )
					{
						double heading = 90.0f - pack->heading;
						double sinY, cosY;
						double sinPit, cosPit;
						fast_sin_cos( degree2rad(heading), &sinY, &cosY );
						fast_sin_cos( degree2rad(pack->pitch), &sinPit, &cosPit );
						
						vector3<double> relPos( 
							pack->length*cosPit*100 * cosY,
							pack->length*cosPit*100 * sinY,  
							pack->length*sinPit*100 );
						if( pack->solStatus==0 && pack->posType==50 )
							available=true;
						DAOSensorUpdate( 0, dao_key, relPos, available );
					}
				}
				else if(gps_state.frame_id == 309)
				{// EVENT����
					last_update_time = TIME::now();
					struct Event_Pack
					{	
						uint32_t itow;
						uint8_t eventID;//�¼����루Event 1 ��Event 2��
						uint8_t status; //�¼�״̬��������  
						uint8_t reserved0;
						uint8_t reserved1;
						uint32_t week;  //��
						uint32_t reserved2;
						uint32_t offset_second;   //����ǰ GGA ���Ƶ�ʣ�EVENT ʱ������ӽ���GGA ����ľ���ʱ��֮���ƫ��ֵ(second)
						uint32_t offset_SubSecond;//����ǰ GGA ���Ƶ�ʣ�EVENT ʱ������ӽ���GGA ����ľ���ʱ��֮���ƫ��ֵ(nanosecond)
						uint16_t prn;
					}__attribute__((packed));	
					Event_Pack* pack = (Event_Pack*)&frame_datas[gps_state.header_length];	
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );						
				}
				else if(gps_state.frame_id == 954)
				{// STADOP DOP��Ϣ
					last_update_time = TIME::now();
					dop_pack = *(Dop_Pack*)&frame_datas[gps_state.header_length];	
				}
				else if(gps_state.frame_id == 101)
				{// RECTIME ��Ϣ
					last_update_time = TIME::now();
					struct Time_Pack
					{	
						uint32_t itow;
						uint32_t clock_status;//ʱ��ģ��״̬
						double offset;//����� GPS ʱ�Ľ��ջ��Ӳ�
						double Offset_std;//GPS ʱ�䵽 UTC ʱ���ƫ
						uint32_t year;//UTC ��  
						uint8_t month;//UTC �� (0-12)
						uint8_t day;	//UTC �� (0-31)
						uint8_t hour; //UTC Сʱ (0-23)  
						uint8_t min;	//UTC ���� (0-59)  
						uint32_t ms;	//UTC ���� (0-60999)
						uint32_t utc_status;//UTC ״̬��0 = INVALID����Ч�� 1 =VALID����Ч�� 2 = WARNING 11 
					}__attribute__((packed));					
					Time_Pack* pack = (Time_Pack*)&frame_datas[gps_state.header_length];	
					
          if( (pack->utc_status & (uint8_t)0x01) == (uint8_t)0x01 )
					{
						addition_inf[2] = *(uint16_t*)&frame_datas[14];
						addition_inf[3] = *(uint32_t*)&frame_datas[16];
					}					
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );							
				}				
				else if(gps_state.frame_id == 43)
				{// ԭʼ�۲�������Ϣ
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );	
					//debug_test[23] =	gps_state.frame_datas_length;
				}
				else if(gps_state.frame_id == 1695)
				{// BDS ��������
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );						
				}
				else if(gps_state.frame_id == 722)
				{// GLONASS ��������
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );						
				}
				else if(gps_state.frame_id == 1330)
				{// QZSS ��������
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );						
				}
				else if(gps_state.frame_id == 25)
				{// GPS ��������
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );						
				}
				else if(gps_state.frame_id == 1335)
				{// HEADING2
					last_update_time = TIME::now();
					struct Heading2_Pack
					{	
						uint32_t solStatus;//λ�ý�״̬
						uint32_t posType;//λ������
						float length;	//���߳� (0 �� 3000 m)  
						float heading;//���� (0 �� 360.0 deg)
						float pitch;	//����(��90 deg)  �ƶ�վ�ڻ�վ�Ϸ�Ϊ��
						float rsv2;
						float hdgstddev;//�����׼ƫ��
						float ptchstddev;//������׼ƫ��
						uint8_t rover_stn_id[4];//��վ ID
						uint8_t master_stn_id[4];//��վ ID
						uint8_t SVTrack;//���ٵ�������  
						uint8_t SVUse;//�ڽ���ʹ�õ�������
						uint8_t obs;//��ֹ�߶Ƚ����ϵ�������
						uint8_t multi;//��ֹ�߶Ƚ�������L2�۲��������
						uint8_t rsv3;
						uint8_t ext_sol_stat;//��չ���״̬
						uint8_t mask1;//Galileo ʹ�õ��ź�����
						uint8_t mask2;//GPS, GLONASS �� BDS ʹ�õ��ź�����						
					}__attribute__((packed));
					Heading2_Pack* pack = (Heading2_Pack*)&frame_datas[gps_state.header_length];
					
					extern float debug_test[30];
					debug_test[27] = pack->posType;
					debug_test[28] += 1;
					
					bool unLocked;
					is_Attitude_Control_Enabled(&unLocked);
					if( unLocked )
					{
						//uint8_t mt_count = get_MainMotorCount();
						double logbuf[4];
						logbuf[0] = pack->posType;
						logbuf[1] = pack->solStatus;
						logbuf[2] = pack->length;
						SDLog_Msg_DebugVect( "shuncheng", logbuf, 3 );
					}
					
					if( pack->solStatus==0 && pack->posType==50 )
					{
						vector3<double> pos;
						double heading = 90.0f - pack->heading;
						double sinY, cosY;
						double sinPit, cosPit;
						fast_sin_cos( degree2rad(heading), &sinY, &cosY );
						fast_sin_cos( degree2rad(pack->pitch), &sinPit, &cosPit );
						pos.x = -pack->length*cosPit*100 * cosY;
						pos.y = -pack->length*cosPit*100 * sinY;
						pos.z = -pack->length*sinPit*100;
						update_precLandSensorENU(precKey, pos, true, true );
					}
				}
			}
			
			if( last_update_time.get_pass_time() > 2 )
			{	//���ղ�������
				PositionSensorUnRegister( default_rtk_sensor_index,sensor_key );
				DAOSensorUnRegister(0,dao_key);
				//�ر�Rtkע��
				RtkPort_setEna( rtk_port_ind, false );
				goto GPS_CheckBaud;
			}	
		}
		else
		{	//���ղ�������
			PositionSensorUnRegister( default_rtk_sensor_index,sensor_key );
			DAOSensorUnRegister(0,dao_key);
			//�ر�Rtkע��
			RtkPort_setEna( rtk_port_ind, false );
			goto GPS_CheckBaud;
		}	
	}		
}

static bool RTK_DriverInit( Port port, uint32_t param )
{
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( RTK_Server, "RTK3", 2000, driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_RTK_UM982_movingTrack()
{
	PortFunc_Register( 18, RTK_DriverInit );
}
