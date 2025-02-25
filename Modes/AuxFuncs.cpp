#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "drv_PWMOut.hpp"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "MeasurementSystem.hpp"
#include "ControlSystem.hpp"
#include "event_groups.h"
#include "semphr.h"
#include "usb_composite.h"
#include "MavlinkCMDProcess.hpp"

// ����֮ǰͨ��ֵ
static double last_Channel_values[16];
// ͨ����ʱ����
static uint32_t channelTemp1[16] = {0};
static float channelTemp2[16] = {0};
static float channelTemp3[16] = {0};
// ��̨�Ƿ��Զ�����
static float GimbalCtrl_LockedAtt[16];

/*��ѥ����*/
// ���մ���
static uint16_t PhotoCnt = 0;
// ��Ƭ���
static uint16_t PhotoIndex = 1;

// ��ʼ�ȴ�������־
static bool waiting_IOTrig = false;
// ������ɱ�־
static EventGroupHandle_t IO_events = xEventGroupCreate();

static SemaphoreHandle_t PosLogMutex = xSemaphoreCreateMutex();
static bool SD_Pos_Record()
{
	// ��ȡʱ��
	RTC_TimeStruct RTC_Time;
	RTC_Time = Get_RTC_Time();
	// ��ȡ�ٶ�
	vector3<double> vel;
	get_VelocityENU_Ctrl(&vel);

	// ��ȡ��̬
	Quaternion airframe_quat;
	get_Attitude_quat(&airframe_quat);
	airframe_quat.Enu2Ned();

	static int8_t global_pos_ind = -1;
	PosSensorHealthInf3 posInf;
	if (global_pos_ind < 0)
	{ // ��һ�λ�ȡ����λ�ô�����
		if (get_OptimalGlobal_XYZ(&posInf))
			global_pos_ind = posInf.sensor_ind;
	}
	else
		get_PosSensorHealth_XYZ(&posInf, global_pos_ind);

	double lat = 0;
	double lon = 0;
	double alt = 0;
	double accN = 999999;
	double accE = 999999;
	double accD = 999999;
	uint8_t fix_type = 0;
	uint16_t week = 0;
	double TOW = 0;

	if (global_pos_ind >= 0)
	{
		Position_Sensor gps_sensor;
		if (GetPositionSensor(global_pos_ind, &gps_sensor))
		{
			if (gps_sensor.data.available && gps_sensor.data.sensor_type == Position_Sensor_Type_GlobalPositioning)
			{
				// ���㾭γ��
				map_projection_reproject(&posInf.mp,
										 posInf.PositionENU.x + posInf.HOffset.x,
										 posInf.PositionENU.y + posInf.HOffset.y,
										 &lat, &lon);
				// �߶�
				alt = posInf.PositionENU.z + posInf.HOffset.z;
				alt *= 0.01;
				// ����
				accN = gps_sensor.inf.addition_inf[4] * 0.01;
				accE = gps_sensor.inf.addition_inf[4] * 0.01;
				accD = gps_sensor.inf.addition_inf[5] * 0.01;
				// week
				week = gps_sensor.inf.addition_inf[2];
				// TOW
				TOW = gps_sensor.inf.addition_inf[3];
				// fix
				if (gps_sensor.inf.addition_inf[1] == 1)
					fix_type = 0;
				else if (gps_sensor.inf.addition_inf[1] == 5)
					fix_type = 34;
				else if (gps_sensor.inf.addition_inf[1] == 6)
					fix_type = 50;
				else
					fix_type = 16;
			}
		}
	}

	xSemaphoreTake(PosLogMutex, portMAX_DELAY);
	uint16_t photo_ind = PhotoIndex;
	char pos_txt_buf[200];
	int n = sprintf(
		pos_txt_buf,
		"%4d\t%6.6f\t[%4d]\t%4d,N\t%4d,E\t%4d,V \t%11.8f,Lat \t%11.8f,Lon \t%6.3f,Ellh \t%9.6f,    %9.6f,    %9.6f,    %2d,Q\r\n",
		PhotoIndex++,
		TOW,
		week,
		0,
		0,
		0,
		lat,
		lon,
		alt,
		accN,
		accE,
		accD,
		fix_type);

	xSemaphoreGive(PosLogMutex);

	if (!SDLog_Txt1(pos_txt_buf, n))
		return false;

	// ����������Ϣ������??
	double Altitude_Local = 0;
	vector3<double> Position;
	get_Position_Ctrl(&Position);
	double homeZ;
	double heightAboveGround = 0;
	if (getHomeLocalZ(&homeZ, 0, 0.01))
		heightAboveGround = Position.z - homeZ;

	mavlink_message_t msg_sd;
	for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
	{ // �������ж˿�
		if (mavlink_lock_chan(i, 2 / configTICK_RATE_HZ))
		{
			mavlink_msg_camera_feedback_pack_chan(
				get_CommulinkSysId(),  // system id
				get_CommulinkCompId(), // component id
				i,
				&msg_sd,
				TIME::get_System_Run_Time() * 1e3, // boot ms
				1,								   // target_system  System ID
				global_pos_ind,					   // camera_id
				photo_ind,						   // Image index
				lat * 1e7,						   // lat [degE7]
				lon * 1e7,						   // lon [degE7]
				alt,							   // alt_msl [m] Altitude (MSL).
				heightAboveGround * 0.01,		   // alt_rel [m] Altitude (Relative to HOME location).
				0,								   // Camera Roll
				0,								   // Camera Pitch
				0,								   // Camera Yaw
				30,								   // foc_len [mm] Focal Length
				0,								   // 0:Shooting photos, not video
				photo_ind						   // Completed image captures
			);
			const Port *port = get_CommuPort(i);
			if (port->write)
			{
				mavlink_msg_to_send_buffer(port->write,
										   port->lock,
										   port->unlock,
										   &msg_sd, 0, 2 / configTICK_RATE_HZ);
			}
			mavlink_unlock_chan(i);
		}
	}
	return true;
}

static void IOTrigTCB(void *pvParameter1, uint32_t ulParameter2)
{
	AuxFuncsConfig aux_configs;
	uint8_t trig_ena[8];
	if (ReadParam("Aux_CamTrigEna", 0, 0, (uint64_t *)trig_ena, 0) == PR_OK)
	{
		if (trig_ena[0])
		{
			if (SD_Pos_Record())
			{
				xEventGroupSetBits(IO_events, 1 << 0);
				if (waiting_IOTrig == false)
					sendLedSignal(LEDSignal_Success1);
			}
		}
	}
}

void wait_IOTrig_start()
{
	waiting_IOTrig = true;
	xEventGroupClearBits(IO_events, 1 << 0);
}

bool wait_IOTrig(double TIME)
{
	uint32_t revents = xEventGroupWaitBits(IO_events, 1 << 0, pdTRUE, pdFALSE, TIME * configTICK_RATE_HZ);
	waiting_IOTrig = false;
	if ((revents & (1 << 0)) == 0)
		return false;
	else
		return true;
}

// IO��ת����??
extern "C" void TIM4_IRQHandler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static TIME last_trig_TIME(false);
	uint32_t SR = TIM4->SR;
	if (SR & (1 << 2))
	{ // ����IO���½���
		uint32_t CCR2 = TIM4->CCR2;
		double pass_t = last_trig_TIME.get_pass_time_st();

		if (pass_t > 0.05)
		{
			xLPDaemonTaskCallFromISR(
				IOTrigTCB,
				0,
				0,
				&xHigherPriorityTaskWoken);
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/*��ѥ����*/

/*��¼Pos*/
bool recordPos(bool acTrig)
{
	if (acTrig)
	{
		xSemaphoreTake(PosLogMutex, portMAX_DELAY);
		++PhotoCnt;
		xSemaphoreGive(PosLogMutex);

		mavlink_message_t msg_sd;
		for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
		{ // �������ж˿�
			if (mavlink_lock_chan(i, 2 / configTICK_RATE_HZ))
			{
				mavlink_msg_camera_status_pack_chan(
					get_CommulinkSysId(),  // system id
					get_CommulinkCompId(), // component id
					i,
					&msg_sd,
					TIME::get_System_Run_Time() * 1e6, // boot us
					255,							   // target_system
					1,								   // cam index
					PhotoCnt,						   // image index
					1,								   // event id
					0,								   // p1
					0,								   // p2
					0,								   // p3
					0								   // p4
				);
				const Port *port = get_CommuPort(i);
				if (port->write)
				{
					mavlink_msg_to_send_buffer(port->write,
											   port->lock,
											   port->unlock,
											   &msg_sd, 0, 2 / configTICK_RATE_HZ);
				}
				mavlink_unlock_chan(i);
			}
		}
	}
	return SD_Pos_Record();
}
/*��¼Pos*/

/*���չ���*/
static SemaphoreHandle_t CamMutex = xSemaphoreCreateRecursiveMutex();
bool AuxCamTakePhoto()
{
	if (getInitializationCompleted() == false)
		return false;
	if (get_is_usb_connected())
		return false;
	uint8_t cam_chans = 0;
	AuxFuncsConfig aux_configs;
	ReadParamGroup("AuxCfg", (uint64_t *)&aux_configs, 0);
	if (xSemaphoreTakeRecursive(CamMutex, 0.1 * configTICK_RATE_HZ))
	{
		uint8_t MainMotorCount = get_MainMotorCount();
		if (aux_configs.Aux_CamTrigEna[0])
			wait_IOTrig_start();

		// ���ͣ����ߣ����PWM
		for (uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i)
		{
			uint16_t aux_cfg = ((uint16_t *)&aux_configs)[i * 4];
			if ((aux_cfg >= 25 && aux_cfg <= 48) || (aux_cfg >= 1025 && aux_cfg <= 1048))
			{
				Aux_PWM_Out(aux_configs.Aux_CamOnPwm[0] * 0.1 - 100, i);
				++cam_chans;
			}
		}

		if (cam_chans == 0)
		{ // ���������
			uint8_t res = 0;

			if (aux_configs.Aux_YTSurveyPic[0] > 0)
			{ // ������̨��Ҫ����
				if (!aux_configs.Aux_CamTrigEna[0])
					wait_IOTrig_start();

				CmdMsg cmd_msg;
				cmd_msg.cmd = MAV_CMD_IMAGE_START_CAPTURE;
				SendCmdMsgToSingleCamera(cmd_msg, 0.01);

				if (aux_configs.Aux_CamTrigEna[0] > 0)
				{ // ������̨��Ҫ�ȴ�trig���Ŵ���
					if (wait_IOTrig(aux_configs.Aux_CamShTime[0] > 1.5 ? 1.5 : aux_configs.Aux_CamShTime[0]))
						res = 10;
				}
				else
				{ // ������̨�ȴ�ָ�����
					wait_IOTrig(0);

					double waitT = 0;
					while (1)
					{
						CmdMsg msg;
						bool msg_rd = ReceiveCmdMsgFromTask(&msg, CAMERA_FEEDBACK_QUEUE_ID, 0.002);
						if (msg_rd)
						{ // ��������ɹ�
							res = msg.cmd;
							break;
						}

						if (waitT > aux_configs.Aux_CamShTime[0] || waitT > 1.5)
							break;
						waitT += 0.01;
						os_delay(0.01);
					}
				}
			}

			xSemaphoreGiveRecursive(CamMutex);
			if (res)
			{
				xSemaphoreTake(PosLogMutex, portMAX_DELAY);
				++PhotoCnt;
				xSemaphoreGive(PosLogMutex);

				mavlink_message_t msg_sd;
				for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
				{ // �������ж˿�
					if (mavlink_lock_chan(i, 2 / configTICK_RATE_HZ))
					{
						mavlink_msg_camera_status_pack_chan(
							get_CommulinkSysId(),  // system id
							get_CommulinkCompId(), // component id
							i,
							&msg_sd,
							TIME::get_System_Run_Time() * 1e6, // boot us
							255,							   // target_system
							1,								   // cam index
							PhotoCnt,						   // image index
							1,								   // event id
							0,								   // p1
							0,								   // p2
							0,								   // p3
							0								   // p4
						);
						const Port *port = get_CommuPort(i);
						if (port->write)
						{
							mavlink_msg_to_send_buffer(port->write,
													   port->lock,
													   port->unlock,
													   &msg_sd, 0, 2 / configTICK_RATE_HZ);
						}
						mavlink_unlock_chan(i);
					}
				}

				// �������ȴ��ڲ����������̼�¼POS
				if (res != 3)
					SD_Pos_Record();

				return true;
			}
			else
				return false;
		}

		// ���ߣ����ͣ����PWM
		os_delay(aux_configs.Aux_CamShTime[0]);
		for (uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i)
		{
			uint16_t aux_cfg = ((uint16_t *)&aux_configs)[i * 4];
			if ((aux_cfg >= 25 && aux_cfg <= 48) || (aux_cfg >= 1025 && aux_cfg <= 1048))
				Aux_PWM_Out(aux_configs.Aux_CamOffPwm[0] * 0.1 - 100, i);
		}

		xSemaphoreTake(PosLogMutex, portMAX_DELAY);
		++PhotoCnt;
		xSemaphoreGive(PosLogMutex);

		mavlink_message_t msg_sd;
		for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
		{ // �������ж˿�
			if (mavlink_lock_chan(i, 2 / configTICK_RATE_HZ))
			{
				mavlink_msg_camera_status_pack_chan(
					get_CommulinkSysId(),  // system id
					get_CommulinkCompId(), // component id
					i,
					&msg_sd,
					TIME::get_System_Run_Time() * 1e6, // boot us
					255,							   // target_system
					1,								   // cam index
					PhotoCnt,						   // image index
					1,								   // event id
					0,								   // p1
					0,								   // p2
					0,								   // p3
					0								   // p4
				);
				const Port *port = get_CommuPort(i);
				if (port->write)
				{
					mavlink_msg_to_send_buffer(port->write,
											   port->lock,
											   port->unlock,
											   &msg_sd, 0, 2 / configTICK_RATE_HZ);
				}
				mavlink_unlock_chan(i);
			}
		}

		if (aux_configs.Aux_CamTrigEna[0])
		{ // �ȴ������ѥ
			if (wait_IOTrig(aux_configs.Aux_CamShTime[0] > 1.5 ? 1.5 : aux_configs.Aux_CamShTime[0]) == false)
			{
				xSemaphoreGiveRecursive(CamMutex);
				return false;
			}
		}

		// �����ѥû��ʹ�������̼�¼POS
		if (!aux_configs.Aux_CamTrigEna[0])
			SD_Pos_Record();

		xSemaphoreGiveRecursive(CamMutex);
		return true;
	}
	else
		return false;
}

static uint8_t camPhotoAsyncStep = 0;
static float camPhotoAsyncT = 0;
static uint16_t lastCamPhotoAsyncPhotoIndex = 0;
#define resetCamPhotoAsyncStep (camPhotoAsyncStep = camPhotoAsyncT = 0)
bool AuxCamTakePhotoAsync(double TIMEOUT)
{
	if (xSemaphoreTakeRecursive(CamMutex, TIMEOUT * configTICK_RATE_HZ))
	{
		resetCamPhotoAsyncStep;
		camPhotoAsyncStep = 1;
		camPhotoAsyncT = 0;
		xSemaphoreGiveRecursive(CamMutex);
		return true;
	}
	return false;
}
static void AuxCamTakePhotoAsyncServer(float h, const AuxFuncsConfig &aux_configs)
{
	if (getInitializationCompleted() == false ||
		get_is_usb_connected())
	{
		if (camPhotoAsyncStep > 1)
			xSemaphoreGiveRecursive(CamMutex);
		camPhotoAsyncStep = camPhotoAsyncT = 0;
		// ���������̨����������
		CmdMsg msg;
		ReceiveCmdMsgFromTask(&msg, CAMERA_FEEDBACK_QUEUE_ID, 0);

		return;
	}

	if (camPhotoAsyncStep == 0)
	{ // ����ʱ����������̨���շ���
		CmdMsg msg;
		bool msg_rd = ReceiveCmdMsgFromTask(&msg, CAMERA_FEEDBACK_QUEUE_ID, 0);
		if (msg_rd && msg.cmd == 1)
		{
			xSemaphoreTake(PosLogMutex, portMAX_DELAY);
			++PhotoCnt;
			xSemaphoreGive(PosLogMutex);
			SD_Pos_Record();
		}
	}

	if (camPhotoAsyncStep > 1)
	{
		camPhotoAsyncT += h;
		if (camPhotoAsyncT > 10)
			resetCamPhotoAsyncStep;
	}

	switch (camPhotoAsyncStep)
	{
	case 1:
	{ // ׼����������
		if (xSemaphoreTakeRecursive(CamMutex, 0.1 * configTICK_RATE_HZ))
		{
			uint8_t cam_chans = 0;
			lastCamPhotoAsyncPhotoIndex = PhotoIndex;

			uint8_t MainMotorCount = get_MainMotorCount();
			// ʹ�����PWM
			for (uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i)
			{
				uint16_t aux_cfg = ((uint16_t *)&aux_configs)[i * 4];
				if ((aux_cfg >= 25 && aux_cfg <= 48) || (aux_cfg >= 1025 && aux_cfg <= 1048))
				{
					Aux_PWM_Out(aux_configs.Aux_CamOnPwm[0] * 0.1 - 100, i);
					++cam_chans;
				}
			}
			camPhotoAsyncT = 0;

			if (cam_chans == 0)
			{ // ���������������̨
				uint8_t res = 0;

				if (aux_configs.Aux_YTSurveyPic[0] > 0)
				{ // ������̨��Ҫ����
					CmdMsg cmd_msg;
					cmd_msg.cmd = MAV_CMD_IMAGE_START_CAPTURE;
					SendCmdMsgToSingleCamera(cmd_msg, 0.01);

					if (aux_configs.Aux_CamTrigEna[0] > 0)
					{ // ������̨��Ҫ�ȴ�trig���Ŵ���
						camPhotoAsyncStep = 20;
					}
					else
					{ // ����������̨�ȴ�ָ�������
						camPhotoAsyncStep = 30;
					}
				}
				else
				{
					resetCamPhotoAsyncStep;
					xSemaphoreGiveRecursive(CamMutex);
				}
			}
			else
			{ // ������Ŵ������
				camPhotoAsyncStep = 10;
			}
		}
		else
			resetCamPhotoAsyncStep;
		break;
	}

	case 10:
	{ // ������Ŵ������
		// ׼��ʧЧ�������

		if (camPhotoAsyncT > aux_configs.Aux_CamShTime[0])
		{
			// ʧЧ���PWM
			uint8_t MainMotorCount = get_MainMotorCount();
			for (uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i)
			{
				uint16_t aux_cfg = ((uint16_t *)&aux_configs)[i * 4];
				if ((aux_cfg >= 25 && aux_cfg <= 48) || (aux_cfg >= 1025 && aux_cfg <= 1048))
					Aux_PWM_Out(aux_configs.Aux_CamOffPwm[0] * 0.1 - 100, i);
			}

			xSemaphoreTake(PosLogMutex, portMAX_DELAY);
			++PhotoCnt;
			xSemaphoreGive(PosLogMutex);

			mavlink_message_t msg_sd;
			for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
			{ // �������ж˿�
				if (mavlink_lock_chan(i, 2 / configTICK_RATE_HZ))
				{
					mavlink_msg_camera_status_pack_chan(
						get_CommulinkSysId(),  // system id
						get_CommulinkCompId(), // component id
						i,
						&msg_sd,
						TIME::get_System_Run_Time() * 1e6, // boot us
						255,							   // target_system
						1,								   // cam index
						PhotoCnt,						   // image index
						1,								   // event id
						0,								   // p1
						0,								   // p2
						0,								   // p3
						0								   // p4
					);
					const Port *port = get_CommuPort(i);
					if (port->write)
					{
						mavlink_msg_to_send_buffer(port->write,
												   port->lock,
												   port->unlock,
												   &msg_sd, 0, 2 / configTICK_RATE_HZ);
					}
					mavlink_unlock_chan(i);
				}
			}

			if (aux_configs.Aux_CamTrigEna[0])
			{ // �ȴ������ѥ
				camPhotoAsyncStep = 20;
			}
			else
			{ // �����ѥû��ʹ�������̼�¼POS
				SD_Pos_Record();
				camPhotoAsyncStep = 11;
				camPhotoAsyncT = 0;
				//						resetCamPhotoAsyncStep;
				//						xSemaphoreGiveRecursive(CamMutex);
			}
		}
		break;
	}
	case 11:
	{ // ��ʱ�ȴ�shʱ��
		if (camPhotoAsyncT > aux_configs.Aux_CamShTime[0])
		{
			resetCamPhotoAsyncStep;
			xSemaphoreGiveRecursive(CamMutex);
		}
		break;
	}

	case 20:
	{ // �ȴ���ѥ����
		if (camPhotoAsyncT > aux_configs.Aux_CamShTime[0] && (lastCamPhotoAsyncPhotoIndex != PhotoIndex || camPhotoAsyncT > 1.5))
		{
			resetCamPhotoAsyncStep;
			xSemaphoreGiveRecursive(CamMutex);
		}
		break;
	}
	case 30:
	{ // ������̨�ȴ�����
		CmdMsg msg;
		bool msg_rd = ReceiveCmdMsgFromTask(&msg, CAMERA_FEEDBACK_QUEUE_ID, 0);
		if (msg_rd)
			SD_Pos_Record();
		if (msg_rd || camPhotoAsyncT > aux_configs.Aux_CamShTime[0] || camPhotoAsyncT > 1.5)
		{
			resetCamPhotoAsyncStep;
			xSemaphoreGiveRecursive(CamMutex);
		}
	}
	}
}
/*���չ���*/

/*��������*/
static float TankRMPercent = 100;
float getTankRMPercent() { return TankRMPercent; }

bool setPump1(float percent)
{
	if (getInitializationCompleted() == false)
		return false;
	if (!isvalid(percent))
		return false;

	AuxFuncsConfig aux_configs;
	ReadParamGroup("AuxCfg", (uint64_t *)&aux_configs, 0);

	uint8_t chCount = 0;
	uint8_t MainMotorCount = get_MainMotorCount();
	for (uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i)
	{
		uint16_t aux_cfg = ((uint16_t *)&aux_configs)[i * 4];
		if (aux_cfg >= 525 && aux_cfg <= 548)
		{
			GimbalCtrl_LockedAtt[i] = percent;
			++chCount;
		}
	}
	return chCount != 0;
}

bool setPump1bySpeed(float sp)
{
	if (getInitializationCompleted() == false)
		return false;
	if (!isvalid(sp))
		return false;

	AuxFuncsConfig aux_configs;
	ReadParamGroup("AuxCfg", (uint64_t *)&aux_configs, 0);

	uint8_t chCount = 0;
	uint8_t MainMotorCount = get_MainMotorCount();
	for (uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i)
	{
		uint16_t aux_cfg = ((uint16_t *)&aux_configs)[i * 4];
		if (aux_cfg >= 525 && aux_cfg <= 548)
		{
			float percent = constrain(sp * aux_configs.Aux_Pump1Sp[0], 0.0f, 100.0f);
			GimbalCtrl_LockedAtt[i] = percent;
			++chCount;
		}
	}
	return chCount != 0;
}
/*��������*/

bool setAuxPWM(float PWMus, uint8_t ind)
{
	if (getInitializationCompleted() == false)
		return false;
	if (PWMus < 0 || PWMus > 100000)
		return false;

	AuxFuncsConfig aux_configs;
	ReadParamGroup("AuxCfg", (uint64_t *)&aux_configs, 0);
	uint16_t aux_cfg = ((uint16_t *)&aux_configs)[ind * 4];
	if ((aux_cfg >= 0 && aux_cfg <= 16) || (aux_cfg >= 1001 && aux_cfg <= 1016))
	{
		GimbalCtrl_LockedAtt[ind] = PWMus;
		return true;
	}
	return false;
}

void init_process_AuxFuncs()
{
	//	Receiver rc;
	//	if( getReceiver(&rc) )
	//	{
	//		//��λ����֮ǰͨ��
	//		for( uint8_t i = 0; i < rc.raw_available_channels; ++i )
	//			last_Channel_values[i] = rc.raw_data[i];
	//		for( uint8_t i = rc.raw_available_channels; i < 16; ++i )
	//			last_Channel_values[i] = -200;
	//	}
	//	else
	//	{
	// ��λ����֮ǰͨ��
	for (uint8_t i = 0; i < 16; ++i)
		last_Channel_values[i] = -200;
	//	}
	// ��λ�˶��Զ����Ʊ�־
	for (uint8_t i = 0; i < 16; ++i)
	{
		channelTemp1[i] = 0;
		channelTemp2[i] = 0;
		channelTemp3[i] = 0;
		GimbalCtrl_LockedAtt[i] = 200000;
	}
}

void process_AuxFuncs(const Receiver *rc, double h)
{
	AuxFuncsConfig aux_configs;
	ReadParamGroup("AuxCfg", (uint64_t *)&aux_configs, 0);

	AuxCamTakePhotoAsyncServer(h, aux_configs);

	uint8_t MainMotorCount = get_MainMotorCount();
	for (uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i)
	{
		uint16_t aux_cfg = ((uint16_t *)&aux_configs)[i * 4];
		float aux_param1 = ((float *)&aux_configs.Aux1Param1)[i * 2];
		float aux_param2 = ((float *)&aux_configs.Aux1Param2)[i * 2];
		if (aux_cfg == 0)
		{ // �޹���(��ͨ��ָ�����PWM���)
			if (GimbalCtrl_LockedAtt[i] < 100000)
				Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - 1000) * 0.1, i);
		}
		if (aux_cfg >= 1 && aux_cfg <= 16)
		{ // ӳ��ң����ͨ��
			bool stLocked = false;
			uint8_t stLock_chan = aux_configs.stLockParam[0] >> 24;
			if (stLock_chan)
			{
				if (rc->raw_available_channels >= stLock_chan)
				{
					float chan = rc->raw_data[stLock_chan - 1];
					if (chan > 68)
						stLocked = true;
				}
			}

			if (stLocked)
			{ // ���������
				if (aux_configs.auxCfg[0] & AUX_CFG_STLOCK_FORCEVALUE_BIT)
				{
					uint32_t value = aux_configs.stLockParam[0] & 0xff;
					Aux_PWM_Out((value - 1000) * 0.1, i);
				}
			}
			else
			{
				if (GimbalCtrl_LockedAtt[i] < 100000)
					Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - 1000) * 0.1, i);
				uint8_t ref_chan = aux_cfg - 1;
				if (rc->connected && rc->raw_available_channels > ref_chan)
				{
					if (fabs(GimbalCtrl_LockedAtt[i]) < 100000 && last_Channel_values[i] > -100)
					{ // Aux�Զ�����
						// ͨ���仯������ֵ�ŵ���Aux
						if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10)
						{
							Aux_PWM_Out((rc->raw_data[ref_chan] - 50.0) * aux_param1 + 50 + aux_param2 * 0.1, i);
							last_Channel_values[i] = rc->raw_data[ref_chan];
							GimbalCtrl_LockedAtt[i] = 200000;
						}
					}
					else
					{ // Aux�ֶ�����
						Aux_PWM_Out((rc->raw_data[ref_chan] - 50.0) * aux_param1 + 50 + aux_param2 * 0.1, i);
						last_Channel_values[i] = rc->raw_data[ref_chan];
					}
				}
				else
					last_Channel_values[i] = -200;
			}
		}
		else if (aux_cfg >= 501 && aux_cfg <= 516)
		{ // ӳ��ң����ͨ������������
			uint8_t ref_chan = aux_cfg - 501;
			if (rc->connected && rc->raw_available_channels > ref_chan)
			{
				if (rc->raw_data[ref_chan] > 50)
				{
					if (aux_param1 > 0)
						Aux_PWM_Out(6000, i);
					else
						Aux_PWM_Out(-100, i);
				}
				else
				{
					if (aux_param1 > 0)
						Aux_PWM_Out(-100, i);
					else
						Aux_PWM_Out(6000, i);
				}
			}
		}
		if (aux_cfg >= 525 && aux_cfg <= 548)
		{ // ����ˮ��
			double max = (aux_configs.Aux_Pump1Max[0] - 1000) * 0.1;
			double min = (aux_configs.Aux_Pump1Min[0] - 1000) * 0.1;
			double st = (aux_configs.Aux_Pump1St[0] - 1000) * 0.1;
			double scale = (max - st) / 100.0;
			if (GimbalCtrl_LockedAtt[i] < 200)
			{ // �Զ�����ˮ��
				if (GimbalCtrl_LockedAtt[i] > -20 && GimbalCtrl_LockedAtt[i] < 120)
				{
					float out = constrain(GimbalCtrl_LockedAtt[i], 0.0f, 100.0f);
					Aux_PWM_Out(out * scale + st, i);
				}
				else
				{ // �ر�ˮ��
					Aux_PWM_Out(min, i);
				}
			}

			uint8_t ref_chan = aux_cfg - 525;
			if (rc->connected && rc->raw_available_channels > ref_chan)
			{
				if (GimbalCtrl_LockedAtt[i] > 200)
				{ // ˮ���ֶ�����
					float out = (rc->raw_data[ref_chan] - 50.0) * aux_param1 + aux_param2;
					out = constrain(out, 0.0f, 100.0f);
					float deadZone = 15;
					if (out < deadZone)
						Aux_PWM_Out(min, i);
					else
						Aux_PWM_Out((out - deadZone) * (100 - st) / (100 - deadZone) + st, i);
				}
			}
			else
			{ // ��ң���ź�
				last_Channel_values[i] = -200;
				if (GimbalCtrl_LockedAtt[i] > 200)
				{ // ���Զ���������ң���ź�
					// �ر�ˮ��
					Aux_PWM_Out(min, i);
				}
			}
		}
		else if (aux_cfg >= 25 && aux_cfg <= 48)
		{ // ��ң������Ӧͨ������������Ŵ���(raw_data)
			if (xSemaphoreTakeRecursive(CamMutex, 0))
			{
				if (camPhotoAsyncStep == 0)
				{
					uint8_t ref_chan = aux_cfg - 25;
					if (rc->connected && rc->raw_available_channels > ref_chan)
					{
						if (last_Channel_values[i] > -100)
						{
							if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 15)
							{ // �������
								if (last_Channel_values[i] > -100)
								{
									AuxCamTakePhoto();
								}
								last_Channel_values[i] = rc->raw_data[ref_chan];
							}
							else
								Aux_PWM_Out(aux_configs.Aux_CamOffPwm[0] * 0.1 - 100, i);
						}
						else
						{ // ��ͨ�����ݲ�����
							Aux_PWM_Out(aux_configs.Aux_CamOffPwm[0] * 0.1 - 100, i);
							last_Channel_values[i] = rc->raw_data[ref_chan];
						}
					}
					else
					{ // ��ң����
						Aux_PWM_Out(aux_configs.Aux_CamOffPwm[0] * 0.1 - 100, i);
						last_Channel_values[i] = -200;
					}
				}
				xSemaphoreGiveRecursive(CamMutex);
			}
		}
		else if (aux_cfg >= 49 && aux_cfg <= 72)
		{ // ��ң������Ӧͨ��������ˢ��̨��������(raw_data)
			double angle90 = (aux_configs.Aux_BsYTPit90[0] - 1000) * 0.1;
			double angle0 = (aux_configs.Aux_BsYTPit0[0] - 1000) * 0.1;
			double scale = (angle90 - angle0) / 90.0;
			if (GimbalCtrl_LockedAtt[i] < 200)
				Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - 0) * scale + angle0, i);

			uint8_t ref_chan = aux_cfg - 49;
			if (rc->connected && rc->raw_available_channels > ref_chan)
			{
				if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
				{ // ��̨�Զ�����
					// ͨ���仯������ֵ�ŵ�����̨
					if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10)
					{
						float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
						angle = constrain(angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0]);
						Aux_PWM_Out((angle - 0) * scale + angle0, i);
						last_Channel_values[i] = rc->raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 200000;
					}
				}
				else
				{ // ��̨�ֶ�����
					float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
					angle = constrain(angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0]);
					Aux_PWM_Out((angle - 0) * scale + angle0, i);
					last_Channel_values[i] = rc->raw_data[ref_chan];
				}
			}
			else
			{ // ��ң���ź�
				last_Channel_values[i] = -200;
				if (GimbalCtrl_LockedAtt[i] > 200)
				{ // ���Զ���������ң���ź�
					// ����0�Ƕ�
					float angle = 0;
					Aux_PWM_Out((angle - 0) * scale + angle0, i);
				}
			}
		}
		else if (aux_cfg >= 73 && aux_cfg <= 96)
		{ // ��ң������Ӧͨ�����ж����̨��������(raw_data)
			double angle90 = (aux_configs.Aux_StYTPit90[0] - 1000) * 0.1;
			double angle0 = (aux_configs.Aux_StYTPit0[0] - 1000) * 0.1;
			double scale = (angle90 - angle0) / 90.0;
			Quaternion quat;
			get_Airframe_quat(&quat);
			double pitch = rad2degree(quat.getPitch());
			if (GimbalCtrl_LockedAtt[i] < 200)
				Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - pitch - 0) * scale + angle0, i);

			uint8_t ref_chan = aux_cfg - 73;
			if (rc->connected && rc->raw_available_channels > ref_chan)
			{
				if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
				{ // ��̨�Զ�����
					// ͨ���仯������ֵ�ŵ�����̨
					if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10)
					{
						float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
						angle -= pitch;
						angle = constrain(angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0]);
						Aux_PWM_Out((angle - 0) * scale + angle0, i);
						last_Channel_values[i] = rc->raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 20000;
					}
				}
				else
				{ // ��̨�ֶ�����
					float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
					angle -= pitch;
					angle = constrain(angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0]);
					Aux_PWM_Out((angle - 0) * scale + angle0, i);
					last_Channel_values[i] = rc->raw_data[ref_chan];
				}
			}
			else
			{ // ��ң���ź�
				last_Channel_values[i] = -200;
				if (GimbalCtrl_LockedAtt[i] > 200)
				{ // ���Զ���������ң���ź�
					// ����0�Ƕ�
					float angle = 0;
					Aux_PWM_Out((angle - pitch - 0) * scale + angle0, i);
				}
			}
		}
		else if (aux_cfg >= 273 && aux_cfg <= 296)
		{ // ��ң������Ӧͨ�����ж����̨2��������(raw_data)
			double angle90 = (aux_configs.Aux_StYT2Pit90[0] - 1000) * 0.1;
			double angle0 = (aux_configs.Aux_StYT2Pit0[0] - 1000) * 0.1;
			double scale = (angle90 - angle0) / 90.0;
			Quaternion quat;
			get_Airframe_quat(&quat);
			double pitch = rad2degree(quat.getPitch());
			if (GimbalCtrl_LockedAtt[i] < 200)
				Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - pitch - 0) * scale + angle0, i);

			uint8_t ref_chan = aux_cfg - 273;
			if (rc->connected && rc->raw_available_channels > ref_chan)
			{
				if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
				{ // ��̨�Զ�����
					// ͨ���仯������ֵ�ŵ�����̨
					if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10)
					{
						float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
						angle -= pitch;
						angle = constrain(angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0]);
						Aux_PWM_Out((angle - 0) * scale + angle0, i);
						last_Channel_values[i] = rc->raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 20000;
					}
				}
				else
				{ // ��̨�ֶ�����
					float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
					angle -= pitch;
					angle = constrain(angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0]);
					Aux_PWM_Out((angle - 0) * scale + angle0, i);
					last_Channel_values[i] = rc->raw_data[ref_chan];
				}
			}
			else
			{ // ��ң���ź�
				last_Channel_values[i] = -200;
				if (GimbalCtrl_LockedAtt[i] > 200)
				{ // ���Զ���������ң���ź�
					// ����0�Ƕ�
					float angle = 0;
					Aux_PWM_Out((angle - pitch - 0) * scale + angle0, i);
				}
			}
		}
		else if (aux_cfg >= 97 && aux_cfg <= 120)
		{ // ��ң������Ӧͨ�����ж����̨�������(raw_data)
			double angleN45 = (aux_configs.Aux_StYTRolN45[0] - 1000) * 0.1;
			double angleP45 = (aux_configs.Aux_StYTRolP45[0] - 1000) * 0.1;
			double angle0 = (angleN45 + angleP45) / 2;
			double scale = (angleP45 - angle0) / 45.0;
			Quaternion quat;
			get_Airframe_quat(&quat);
			double roll = rad2degree(quat.getRoll());
			if (GimbalCtrl_LockedAtt[i] < 200)
				Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - roll - 0) * scale + angleN45, i);

			uint8_t ref_chan = aux_cfg - 97;
			if (rc->connected && rc->raw_available_channels > ref_chan)
			{
				if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
				{ // ��̨�Զ�����
					// ͨ���仯������ֵ�ŵ�����̨
					if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10)
					{
						float angle = (rc->raw_data[ref_chan] - 50.0) * (45.0 / 50.0) * aux_param1 + aux_param2;
						angle -= roll;
						angle = constrain(angle, aux_configs.Aux_YTRollMax[0]);
						Aux_PWM_Out((angle - 0) * scale + angle0, i);
						last_Channel_values[i] = rc->raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 20000;
					}
				}
				else
				{ // ��̨�ֶ�����
					float angle = (rc->raw_data[ref_chan] - 50.0) * (45.0 / 50.0) * aux_param1 + aux_param2;
					angle -= roll;
					angle = constrain(angle, aux_configs.Aux_YTRollMax[0]);
					Aux_PWM_Out((angle - 0) * scale + angle0, i);
					last_Channel_values[i] = rc->raw_data[ref_chan];
				}
			}
			else
			{ // ��ң�����ź�
				if (GimbalCtrl_LockedAtt[i] > 200)
				{ // ���Զ���������ң���ź�
					// ����0�Ƕ�
					float angle = 0;
					Aux_PWM_Out((angle - roll - 0) * scale + angle0, i);
				}
			}
		}
		else if (aux_cfg >= 121 && aux_cfg <= 137)
		{ // ������̨���ƣ����߳��д���������ɾ��
		}
		else if (aux_cfg >= 173 && aux_cfg <= 196)
		{ // ��ң������Ӧͨ�����ж����̨������������(raw_data)
			double angle90 = (aux_configs.Aux_StYTPit90[0] - 1000) * 0.1;
			double angle0 = (aux_configs.Aux_StYTPit0[0] - 1000) * 0.1;
			double scale = (angle90 - angle0) / 90.0;

			uint32_t *speedSign = &channelTemp1[i];
			float *speed = &channelTemp2[i];
			float *currentAngle = &channelTemp3[i];

			uint8_t ref_chan = aux_cfg - 173;
			if (rc->connected && rc->raw_available_channels > ref_chan)
			{
				*speed = rc->raw_data[ref_chan] * aux_param1 + aux_param2;
				if (*speed < 0)
					*speed = 0;
			}

			if (*speedSign)
				*currentAngle += (*speed) * h;
			else
				*currentAngle -= (*speed) * h;
			if (*currentAngle >= 90)
			{
				*currentAngle = 90;
				*speedSign = 0;
			}
			else if (*currentAngle <= 0)
			{
				*currentAngle = 0;
				*speedSign = 1;
			}
			Aux_PWM_Out((*currentAngle) * scale + angle0, i);
		}
		else if (aux_cfg >= 197 && aux_cfg <= 220)
		{ // ��ң������Ӧͨ�����ж����̨�����������(raw_data)
			double angleN45 = (aux_configs.Aux_StYTRolN45[0] - 1000) * 0.1;
			double angleP45 = (aux_configs.Aux_StYTRolP45[0] - 1000) * 0.1;
			double angle0 = (angleN45 + angleP45) / 2;
			double scale = (angleP45 - angle0) / 45.0;

			uint32_t *speedSign = &channelTemp1[i];
			float *speed = &channelTemp2[i];
			float *currentAngle = &channelTemp3[i];

			uint8_t ref_chan = aux_cfg - 197;
			if (rc->connected && rc->raw_available_channels > ref_chan)
			{
				*speed = rc->raw_data[ref_chan] * aux_param1 + aux_param2;
				if (*speed < 0)
					*speed = 0;
			}

			if (*speedSign)
				*currentAngle += (*speed) * h;
			else
				*currentAngle -= (*speed) * h;
			if (*currentAngle >= 45)
			{
				*currentAngle = 45;
				*speedSign = 0;
			}
			else if (*currentAngle <= -45)
			{
				*currentAngle = -45;
				*speedSign = 1;
			}

			Aux_PWM_Out((*currentAngle) * scale + angle0, i);
		}

		else if (aux_cfg == 950)
		{ // ��ҩ����
			bool chanValue;
			Aux_ChannelRead(i, &chanValue);
			if (aux_param1 < 0)
				chanValue = !chanValue;
			if (chanValue && last_Channel_values[i] >= 0)
			{ // ��ҩ���ش���
				if (last_Channel_values[i] > 50)
					TankRMPercent = -20;
				else
					last_Channel_values[i] += 1;
			}
			else if (chanValue == false && last_Channel_values[i] <= 0)
			{ // ��ҩ����δ����
				if (last_Channel_values[i] < -50)
				{
					if (TankRMPercent <= 0)
						TankRMPercent = 100;
				}
				else
					last_Channel_values[i] -= 1;
			}
			else
				last_Channel_values[i] = 0;
		}

		else if (aux_cfg >= 1001 && aux_cfg <= 1016)
		{ // ӳ������ҡ��ͨ��
			Receiver jrc;
			getJoyStick(&jrc, 0);

			if (GimbalCtrl_LockedAtt[i] < 100000)
				Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - 1000) * 0.1, i);
			uint8_t ref_chan = aux_cfg - 1001;
			if (jrc.connected && jrc.raw_available_channels > ref_chan)
			{
				if (fabs(GimbalCtrl_LockedAtt[i]) < 100000 && last_Channel_values[i] > -100)
				{ // Aux�Զ�����
					// ͨ���仯������ֵ�ŵ���Aux
					if (fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10)
					{
						Aux_PWM_Out((jrc.raw_data[ref_chan] - 50.0) * aux_param1 + 50 + aux_param2 * 0.1, i);
						last_Channel_values[i] = jrc.raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 200000;
					}
				}
				else
				{ // Aux�ֶ�����
					Aux_PWM_Out((jrc.raw_data[ref_chan] - 50.0) * aux_param1 + 50 + aux_param2 * 0.1, i);
					last_Channel_values[i] = jrc.raw_data[ref_chan];
				}
			}
			else
				last_Channel_values[i] = -200;
		}
		else if (aux_cfg >= 1025 && aux_cfg <= 1048)
		{ // ������ҡ�˶�Ӧͨ������������Ŵ���(raw_data)
			if (xSemaphoreTakeRecursive(CamMutex, 0))
			{
				if (camPhotoAsyncStep == 0)
				{
					Receiver jrc;
					getJoyStick(&jrc, 0);

					uint8_t ref_chan = aux_cfg - 1025;
					if (jrc.connected && jrc.raw_available_channels > ref_chan)
					{
						if (last_Channel_values[i] > -100)
						{
							if (fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 15)
							{ // �������
								if (last_Channel_values[i] > -100)
								{
									AuxCamTakePhoto();
								}
								last_Channel_values[i] = jrc.raw_data[ref_chan];
							}
							else
								Aux_PWM_Out(aux_configs.Aux_CamOffPwm[0] * 0.1 - 100, i);
						}
						else
						{ // ��ͨ�����ݲ�����
							Aux_PWM_Out(aux_configs.Aux_CamOffPwm[0] * 0.1 - 100, i);
							last_Channel_values[i] = jrc.raw_data[ref_chan];
						}
					}
					else
					{ // ��ң����
						Aux_PWM_Out(aux_configs.Aux_CamOffPwm[0] * 0.1 - 100, i);
						last_Channel_values[i] = -200;
					}
				}
				xSemaphoreGiveRecursive(CamMutex);
			}
		}
		else if (aux_cfg >= 1049 && aux_cfg <= 1072)
		{ // ������ҡ�˶�Ӧͨ��������ˢ��̨��������(raw_data)
			Receiver jrc;
			getJoyStick(&jrc, 0);

			double angle90 = (aux_configs.Aux_BsYTPit90[0] - 1000) * 0.1;
			double angle0 = (aux_configs.Aux_BsYTPit0[0] - 1000) * 0.1;
			double scale = (angle90 - angle0) / 90.0;
			if (GimbalCtrl_LockedAtt[i] < 200)
				Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - 0) * scale + angle0, i);

			uint8_t ref_chan = aux_cfg - 1049;
			if (jrc.connected && jrc.raw_available_channels > ref_chan)
			{
				if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
				{ // ��̨�Զ�����
					// ͨ���仯������ֵ�ŵ�����̨
					if (fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10)
					{
						float angle = (jrc.raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
						angle = constrain(angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0]);
						Aux_PWM_Out((angle - 0) * scale + angle0, i);
						last_Channel_values[i] = jrc.raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 200000;
					}
				}
				else
				{ // ��̨�ֶ�����
					float angle = (jrc.raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
					angle = constrain(angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0]);
					Aux_PWM_Out((angle - 0) * scale + angle0, i);
					last_Channel_values[i] = jrc.raw_data[ref_chan];
				}
			}
			else
			{ // ��ң���ź�
				last_Channel_values[i] = -200;
				if (GimbalCtrl_LockedAtt[i] > 200)
				{ // ���Զ���������ң���Ņ�
					// ����0�Ƕ�
					float angle = 0;
					Aux_PWM_Out((angle - 0) * scale + angle0, i);
				}
			}
		}
		else if (aux_cfg >= 1073 && aux_cfg <= 1096)
		{ // ������ҡ�˶�Ӧͨ�����ж����̨��������(raw_data)
			Receiver jrc;
			getJoyStick(&jrc, 0);

			double angle90 = (aux_configs.Aux_StYTPit90[0] - 1000) * 0.1;
			double angle0 = (aux_configs.Aux_StYTPit0[0] - 1000) * 0.1;
			double scale = (angle90 - angle0) / 90.0;
			Quaternion quat;
			get_Airframe_quat(&quat);
			double pitch = rad2degree(quat.getPitch());
			if (GimbalCtrl_LockedAtt[i] < 200)
				Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - pitch - 0) * scale + angle0, i);

			uint8_t ref_chan = aux_cfg - 1073;
			if (jrc.connected && jrc.raw_available_channels > ref_chan)
			{
				if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
				{ // ��̨�Զ�����
					// ͨ���仯������ֵ�ŵ�����̨
					if (fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10)
					{
						float angle = (jrc.raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
						angle -= pitch;
						angle = constrain(angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0]);
						Aux_PWM_Out((angle - 0) * scale + angle0, i);
						last_Channel_values[i] = jrc.raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 20000;
					}
				}
				else
				{ // ��̨�ֶ�����
					float angle = (jrc.raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
					angle -= pitch;
					angle = constrain(angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0]);
					Aux_PWM_Out((angle - 0) * scale + angle0, i);
					last_Channel_values[i] = jrc.raw_data[ref_chan];
				}
			}
			else
			{ // ��ң���ź�
				last_Channel_values[i] = -200;
				if (GimbalCtrl_LockedAtt[i] > 200)
				{ // ���Զ���������ң���Ņ�
					// ����0�Ƕ�
					float angle = 0;
					Aux_PWM_Out((angle - pitch - 0) * scale + angle0, i);
				}
			}
		}
		else if (aux_cfg >= 1097 && aux_cfg <= 1120)
		{ // ������ҡ�˶�Ӧͨ�����ж����̨�������(raw_data)
			Receiver jrc;
			getJoyStick(&jrc, 0);

			double angleN45 = (aux_configs.Aux_StYTRolN45[0] - 1000) * 0.1;
			double angleP45 = (aux_configs.Aux_StYTRolP45[0] - 1000) * 0.1;
			double angle0 = (angleN45 + angleP45) / 2;
			double scale = (angleP45 - angle0) / 45.0;
			Quaternion quat;
			get_Airframe_quat(&quat);
			double roll = rad2degree(quat.getRoll());
			if (GimbalCtrl_LockedAtt[i] < 200)
				Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - roll - 0) * scale + angleN45, i);

			uint8_t ref_chan = aux_cfg - 97;
			if (jrc.connected && jrc.raw_available_channels > ref_chan)
			{
				if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
				{ // ��̨�Զ�����
					// ͨ���仯������ֵ�ŵ�����̨
					if (fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10)
					{
						float angle = (jrc.raw_data[ref_chan] - 50.0) * (45.0 / 50.0) * aux_param1 + aux_param2;
						angle -= roll;
						angle = constrain(angle, aux_configs.Aux_YTRollMax[0]);
						Aux_PWM_Out((angle - 0) * scale + angle0, i);
						last_Channel_values[i] = jrc.raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 20000;
					}
				}
				else
				{ // ��̨�ֶ�����
					float angle = (jrc.raw_data[ref_chan] - 50.0) * (45.0 / 50.0) * aux_param1 + aux_param2;
					angle -= roll;
					angle = constrain(angle, aux_configs.Aux_YTRollMax[0]);
					Aux_PWM_Out((angle - 0) * scale + angle0, i);
					last_Channel_values[i] = jrc.raw_data[ref_chan];
				}
			}
			else
			{ // ��ң�����ź�
				if (GimbalCtrl_LockedAtt[i] > 200)
				{ // ���Զ���������ң���Ņ�
					// ����0�Ƕ�
					float angle = 0;
					Aux_PWM_Out((angle - roll - 0) * scale + angle0, i);
				}
			}
		}
	}

	//	/*������̨�ӿ�*/
	//		for( uint8_t i = 0; i < 8; ++i )
	//		{	//��� ���� ƫ�� ��??
	//			//ģʽ �۽� ����¼�� ��λ
	//			uint16_t aux_cfg = ((uint16_t*)&aux_configs.DgYTRFunc)[i*4];
	//			float aux_param1 = ((float*)&aux_configs.DgYTRParam1)[i*2];
	//			float aux_param2 = ((float*)&aux_configs.DgYTRParam2)[i*2];
	//
	//			if( aux_cfg>=1 && aux_cfg<=16 )
	//			{	//ң����ͨ�����ƽǶ�
	//				if( rc->connected )
	//				{
	//					uint8_t ref_chan = aux_cfg - 1;
	//					if( rc->raw_available_channels > ref_chan )
	//					{
	//						float value;
	//						if( i == DgYTZoom )	//����ͨ�����⴦��
	//							value = rc->raw_data[ref_chan]*0.3*aux_param1+aux_param2*0.1;
	//						else
	//							value = (rc->raw_data[ref_chan]-50.0)*aux_param1+aux_param2*0.1;
	//
	//						set_DgYTCtrl( i,
	//							0, //0-�Ƕ�ģʽ 1-���ٶ�ģʽ
	//							value,
	//							0.1 );
	//					}
	//				}
	//			}
	//			else if( aux_cfg>=25 && aux_cfg<=48 )
	//			{	//ң����ͨ�����ƽǶ�
	//				if( rc->connected )
	//				{
	//					uint8_t ref_chan = aux_cfg - 25;
	//					if( rc->raw_available_channels > ref_chan )
	//					{
	//						set_DgYTCtrl( i,
	//							1, //0-�Ƕ�ģʽ 1-���ٶ�ģʽ
	//							(rc->raw_data[ref_chan]-50.0)*aux_param1+aux_param2*0.1,
	//							0.1 );
	//					}
	//				}
	//			}
	//		}
	//	/*������̨�ӿ�*/
}

bool AuxGimbalSetAngle(double angle)
{
	AuxFuncsConfig aux_configs;
	ReadParamGroup("AuxCfg", (uint64_t *)&aux_configs, 0);

	uint8_t MainMotorsCount = get_MainMotorCount();
	uint8_t AuxChannelsCount = get_AuxChannelCount();
	for (uint8_t i = MainMotorsCount; i < MainMotorsCount + AuxChannelsCount; ++i)
	{
		uint8_t aux_cfg = ((uint8_t *)&aux_configs)[i * 8];
		if (aux_cfg >= 49 && aux_cfg <= 72)
		{ // ��ˢ��̨����??
			angle = constrain(angle, (double)aux_configs.Aux_YTPitMin[0], (double)aux_configs.Aux_YTPitMax[0]);
			GimbalCtrl_LockedAtt[i] = angle;
		}
		else if (aux_cfg >= 73 && aux_cfg <= 96)
		{ // ��ˢ��̨����??
			angle = constrain(angle, (double)aux_configs.Aux_YTPitMin[0], (double)aux_configs.Aux_YTPitMax[0]);
			GimbalCtrl_LockedAtt[i] = angle;
		}
		else if (aux_cfg >= 97 && aux_cfg <= 120)
		{ // ��ˢ��̨����??
			angle = constrain(angle, (double)aux_configs.Aux_YTRollMax[0]);
			GimbalCtrl_LockedAtt[i] = angle;
		}
	}
	return true;
}

void init_AuxFuncs()
{
	// ע����Ϣ����
	TaskQueueRegister(CAMERA_FEEDBACK_QUEUE_ID, 30);

	// ע��ͨ�Ų���
	AuxFuncsConfig initial_cfg;
	initial_cfg.Aux1Func[0] = 0;
	initial_cfg.Aux2Func[0] = 0;
	initial_cfg.Aux3Func[0] = 0;
	initial_cfg.Aux4Func[0] = 0;
	initial_cfg.Aux5Func[0] = 0;
	initial_cfg.Aux6Func[0] = 0;
	initial_cfg.Aux7Func[0] = 0;
	initial_cfg.Aux8Func[0] = 0;
	initial_cfg.Aux9Func[0] = 0;
	initial_cfg.Aux10Func[0] = 0;
	initial_cfg.Aux11Func[0] = 0;
	initial_cfg.Aux12Func[0] = 0;
	initial_cfg.Aux13Func[0] = 0;
	initial_cfg.Aux14Func[0] = 0;

	/* ң��ͨ��1-16��Ӧ��������̨���� */
	initial_cfg.RcYT1Func[0] = 0;
	initial_cfg.RcYT2Func[0] = 0;
	initial_cfg.RcYT3Func[0] = 0;
	initial_cfg.RcYT4Func[0] = 0;
	initial_cfg.RcYT5Func[0] = 0;
	initial_cfg.RcYT6Func[0] = 0;
	initial_cfg.RcYT7Func[0] = 0;
	initial_cfg.RcYT8Func[0] = 0;
	initial_cfg.RcYT9Func[0] = 0;
	initial_cfg.RcYT10Func[0] = 0;
	initial_cfg.RcYT11Func[0] = 0;
	initial_cfg.RcYT12Func[0] = 0;
	initial_cfg.RcYT13Func[0] = 0;
	initial_cfg.RcYT14Func[0] = 0;
	initial_cfg.RcYT15Func[0] = 0;
	initial_cfg.RcYT16Func[0] = 0;

	initial_cfg.Aux1Param1[0] = 1;
	initial_cfg.Aux2Param1[0] = 1;
	initial_cfg.Aux3Param1[0] = 1;
	initial_cfg.Aux4Param1[0] = 1;
	initial_cfg.Aux5Param1[0] = 1;
	initial_cfg.Aux6Param1[0] = 1;
	initial_cfg.Aux7Param1[0] = 1;
	initial_cfg.Aux8Param1[0] = 1;
	initial_cfg.Aux9Param1[0] = 1;
	initial_cfg.Aux10Param1[0] = 1;
	initial_cfg.Aux11Param1[0] = 1;
	initial_cfg.Aux12Param1[0] = 1;
	initial_cfg.Aux13Param1[0] = 1;
	initial_cfg.Aux14Param1[0] = 1;

	/* ң��ͨ��1-16��Ӧ��������̨����1 */
	initial_cfg.RcYT1Param1[0] = 1;
	initial_cfg.RcYT2Param1[0] = 1;
	initial_cfg.RcYT3Param1[0] = 1;
	initial_cfg.RcYT4Param1[0] = 1;
	initial_cfg.RcYT5Param1[0] = 1;
	initial_cfg.RcYT6Param1[0] = 1;
	initial_cfg.RcYT7Param1[0] = 1;
	initial_cfg.RcYT8Param1[0] = 1;
	initial_cfg.RcYT9Param1[0] = 1;
	initial_cfg.RcYT10Param1[0] = 1;
	initial_cfg.RcYT11Param1[0] = 1;
	initial_cfg.RcYT12Param1[0] = 1;
	initial_cfg.RcYT13Param1[0] = 1;
	initial_cfg.RcYT14Param1[0] = 1;
	initial_cfg.RcYT15Param1[0] = 1;
	initial_cfg.RcYT16Param1[0] = 1;

	initial_cfg.Aux1Param2[0] = 0;
	initial_cfg.Aux2Param2[0] = 0;
	initial_cfg.Aux3Param2[0] = 0;
	initial_cfg.Aux4Param2[0] = 0;
	initial_cfg.Aux5Param2[0] = 0;
	initial_cfg.Aux6Param2[0] = 0;
	initial_cfg.Aux7Param2[0] = 0;
	initial_cfg.Aux8Param2[0] = 0;
	initial_cfg.Aux9Param2[0] = 0;
	initial_cfg.Aux10Param2[0] = 0;
	initial_cfg.Aux11Param2[0] = 0;
	initial_cfg.Aux12Param2[0] = 0;
	initial_cfg.Aux13Param2[0] = 0;
	initial_cfg.Aux14Param2[0] = 0;

	/* ң��ͨ��1-16��Ӧ��������̨����2 */
	initial_cfg.RcYT1Param2[0] = 1;
	initial_cfg.RcYT2Param2[0] = 1;
	initial_cfg.RcYT3Param2[0] = 1;
	initial_cfg.RcYT4Param2[0] = 1;
	initial_cfg.RcYT5Param2[0] = 1;
	initial_cfg.RcYT6Param2[0] = 1;
	initial_cfg.RcYT7Param2[0] = 1;
	initial_cfg.RcYT8Param2[0] = 1;
	initial_cfg.RcYT9Param2[0] = 1;
	initial_cfg.RcYT10Param2[0] = 1;
	initial_cfg.RcYT11Param2[0] = 1;
	initial_cfg.RcYT12Param2[0] = 1;
	initial_cfg.RcYT13Param2[0] = 1;
	initial_cfg.RcYT14Param2[0] = 1;
	initial_cfg.RcYT15Param2[0] = 1;
	initial_cfg.RcYT16Param2[0] = 1;

	initial_cfg.auxCfg[0] = 0;
	initial_cfg.stLockParam[0] = 1000;

	initial_cfg.Aux_YTSurveyPic[0] = 0;
	initial_cfg.Aux_CamOnPwm[0] = 2000;
	initial_cfg.Aux_CamOffPwm[0] = 1000;
	initial_cfg.Aux_CamShTime[0] = 0.1;
	initial_cfg.Aux_CamTrigEna[0] = 0;
	initial_cfg.Aux_BsYTPit0[0] = 1000;
	initial_cfg.Aux_BsYTPit90[0] = 2000;
	initial_cfg.Aux_StYTPit0[0] = 1250;
	initial_cfg.Aux_StYTPit90[0] = 1750;
	initial_cfg.Aux_StYT2Pit0[0] = 1250;
	initial_cfg.Aux_StYT2Pit90[0] = 1750;
	initial_cfg.Aux_StYTRolN45[0] = 1000;
	initial_cfg.Aux_StYTRolP45[0] = 2000;
	initial_cfg.Aux_YTPitMin[0] = -20;
	initial_cfg.Aux_YTPitMax[0] = 120;
	initial_cfg.Aux_YTRollMax[0] = 45;
	initial_cfg.Aux_Pump1Min[0] = 1000;
	initial_cfg.Aux_Pump1St[0] = 1100;
	initial_cfg.Aux_Pump1Max[0] = 1950;
	initial_cfg.Aux_Pump1Sp[0] = 0.15;
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT16, // Aux1Func
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16, // Aux14Func

		MAV_PARAM_TYPE_UINT16, // RcYT1Func
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16, // RcYT16Func

		MAV_PARAM_TYPE_REAL32, // Aux1Param1
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32, // Aux14Param1

		MAV_PARAM_TYPE_REAL32, // RcYT1Param1
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32, // RcYT16Param1

		MAV_PARAM_TYPE_REAL32, // Aux1Param2
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32, // Aux14Param2

		MAV_PARAM_TYPE_REAL32, // RcYT1Param2
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32, // RcYT16Param2

		MAV_PARAM_TYPE_UINT32, // auxCfg
		MAV_PARAM_TYPE_UINT32,

		MAV_PARAM_TYPE_UINT8, // Aux_YTSurveyPic
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_UINT8,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_UINT16,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
		MAV_PARAM_TYPE_REAL32,
	};
	SName param_names[] = {
		"Aux_1Func",
		"Aux_2Func",
		"Aux_3Func",
		"Aux_4Func",
		"Aux_5Func",
		"Aux_6Func",
		"Aux_7Func",
		"Aux_8Func",
		"Aux_9Func",
		"Aux_10Func",
		"Aux_11Func",
		"Aux_12Func",
		"Aux_13Func",
		"Aux_14Func",
		"Aux_RcYT1Func",
		"Aux_RcYT2Func",
		"Aux_RcYT3Func",
		"Aux_RcYT4Func",
		"Aux_RcYT5Func",
		"Aux_RcYT6Func",
		"Aux_RcYT7Func",
		"Aux_RcYT8Func",
		"Aux_RcYT9Func",
		"Aux_RcYT10Func",
		"Aux_RcYT11Func",
		"Aux_RcYT12Func",
		"Aux_RcYT13Func",
		"Aux_RcYT14Func",
		"Aux_RcYT15Func",
		"Aux_RcYT16Func",
		"Aux_1Param1",
		"Aux_2Param1",
		"Aux_3Param1",
		"Aux_4Param1",
		"Aux_5Param1",
		"Aux_6Param1",
		"Aux_7Param1",
		"Aux_8Param1",
		"Aux_9Param1",
		"Aux_10Param1",
		"Aux_11Param1",
		"Aux_12Param1",
		"Aux_13Param1",
		"Aux_14Param1",
		"Aux_RcYT1Param1",
		"Aux_RcYT2Param1",
		"Aux_RcYT3Param1",
		"Aux_RcYT4Param1",
		"Aux_RcYT5Param1",
		"Aux_RcYT6Param1",
		"Aux_RcYT7Param1",
		"Aux_RcYT8Param1",
		"Aux_RcYT9Param1",
		"Aux_RcYT10Param1",
		"Aux_RcYT11Param1",
		"Aux_RcYT12Param1",
		"Aux_RcYT13Param1",
		"Aux_RcYT14Param1",
		"Aux_RcYT15Param1",
		"Aux_RcYT16Param1",
		"Aux_1Param2",
		"Aux_2Param2",
		"Aux_3Param2",
		"Aux_4Param2",
		"Aux_5Param2",
		"Aux_6Param2",
		"Aux_7Param2",
		"Aux_8Param2",
		"Aux_9Param2",
		"Aux_10Param2",
		"Aux_11Param2",
		"Aux_12Param2",
		"Aux_13Param2",
		"Aux_14Param2",
		"Aux_RcYT1Param2",
		"Aux_RcYT2Param2",
		"Aux_RcYT3Param2",
		"Aux_RcYT4Param2",
		"Aux_RcYT5Param2",
		"Aux_RcYT6Param2",
		"Aux_RcYT7Param2",
		"Aux_RcYT8Param2",
		"Aux_RcYT9Param2",
		"Aux_RcYT10Param2",
		"Aux_RcYT11Param2",
		"Aux_RcYT12Param2",
		"Aux_RcYT13Param2",
		"Aux_RcYT14Param2",
		"Aux_RcYT15Param2",
		"Aux_RcYT16Param2",

		"Aux_cfg",
		"Aux_stLockParam",

		"Aux_YTSurveyPic", // ������̨�Ƿ����ڲ���������������
		"Aux_CamOnPwm",	   // ���տ�
		"Aux_CamOffPwm",   // ���չ�
		"Aux_CamShTime",   // ���ճ���ʱ��
		"Aux_CamTrigEna",  // ��ѥ����ʹ��
		"Aux_BsYTPit0",
		"Aux_BsYTPit90",
		"Aux_StYTPit0",
		"Aux_StYTPit90",
		"Aux_StYT2Pit0",
		"Aux_StYT2Pit90",
		"Aux_StYTRolN45",
		"Aux_StYTRolP45",
		"Aux_StPitMin",
		"Aux_StPitMax",
		"Aux_StRolMax",
		"Aux_Pump1Min",
		"Aux_Pump1St",
		"Aux_Pump1Max",
		"Aux_Pump1Sp",
	};
	ParamGroupRegister("AuxCfg", 1, sizeof(initial_cfg) / 8, param_types, param_names, (uint64_t *)&initial_cfg);
}
