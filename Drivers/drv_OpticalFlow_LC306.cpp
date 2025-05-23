#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
	uint32_t sensor_key;
};

#define SENSOR_IIC_ADDR 0xdc
const static uint8_t tab_focus[4] = {0x96, 0x26, 0xbc, 0x50};
const static uint8_t Sensor_cfg[] = {
	// 地址, 数据
	0x12, 0x80,
	0x11, 0x30,
	0x1b, 0x06,
	0x6b, 0x43,
	0x12, 0x20,
	0x3a, 0x00,
	0x15, 0x02,
	0x62, 0x81,
	0x08, 0xa0,
	0x06, 0x68,
	0x2b, 0x20,
	0x92, 0x25,
	0x27, 0x97,
	0x17, 0x01,
	0x18, 0x79,
	0x19, 0x00,
	0x1a, 0xa0,
	0x03, 0x00,
	0x13, 0x00,
	0x01, 0x13,
	0x02, 0x20,
	0x87, 0x16,
	0x8c, 0x01,
	0x8d, 0xcc,
	0x13, 0x07,
	0x33, 0x10,
	0x34, 0x1d,
	0x35, 0x46,
	0x36, 0x40,
	0x37, 0xa4,
	0x38, 0x7c,
	0x65, 0x46,
	0x66, 0x46,
	0x6e, 0x20,
	0x9b, 0xa4,
	0x9c, 0x7c,
	0xbc, 0x0c,
	0xbd, 0xa4,
	0xbe, 0x7c,
	0x20, 0x09,
	0x09, 0x03,
	0x72, 0x2f,
	0x73, 0x2f,
	0x74, 0xa7,
	0x75, 0x12,
	0x79, 0x8d,
	0x7a, 0x00,
	0x7e, 0xfa,
	0x70, 0x0f,
	0x7c, 0x84,
	0x7d, 0xba,
	0x5b, 0xc2,
	0x76, 0x90,
	0x7b, 0x55,
	0x71, 0x46,
	0x77, 0xdd,
	0x13, 0x0f,
	0x8a, 0x10,
	0x8b, 0x20,
	0x8e, 0x21,
	0x8f, 0x40,
	0x94, 0x41,
	0x95, 0x7e,
	0x96, 0x7f,
	0x97, 0xf3,
	0x13, 0x07,
	0x24, 0x58,
	0x97, 0x48,
	0x25, 0x08,
	0x94, 0xb5,
	0x95, 0xc0,
	0x80, 0xf4,
	0x81, 0xe0,
	0x82, 0x1b,
	0x83, 0x37,
	0x84, 0x39,
	0x85, 0x58,
	0x86, 0xff,
	0x89, 0x15,
	0x8a, 0xb8,
	0x8b, 0x99,
	0x39, 0x98,
	0x3f, 0x98,
	0x90, 0xa0,
	0x91, 0xe0,
	0x40, 0x20,
	0x41, 0x28,
	0x42, 0x26,
	0x43, 0x25,
	0x44, 0x1f,
	0x45, 0x1a,
	0x46, 0x16,
	0x47, 0x12,
	0x48, 0x0f,
	0x49, 0x0d,
	0x4b, 0x0b,
	0x4c, 0x0a,
	0x4e, 0x08,
	0x4f, 0x06,
	0x50, 0x06,
	0x5a, 0x56,
	0x51, 0x1b,
	0x52, 0x04,
	0x53, 0x4a,
	0x54, 0x26,
	0x57, 0x75,
	0x58, 0x2b,
	0x5a, 0xd6,
	0x51, 0x28,
	0x52, 0x1e,
	0x53, 0x9e,
	0x54, 0x70,
	0x57, 0x50,
	0x58, 0x07,
	0x5c, 0x28,
	0xb0, 0xe0,
	0xb1, 0xc0,
	0xb2, 0xb0,
	0xb3, 0x4f,
	0xb4, 0x63,
	0xb4, 0xe3,
	0xb1, 0xf0,
	0xb2, 0xa0,
	0x55, 0x00,
	0x56, 0x40,
	0x96, 0x50,
	0x9a, 0x30,
	0x6a, 0x81,
	0x23, 0x33,
	0xa0, 0xd0,
	0xa1, 0x31,
	0xa6, 0x04,
	0xa2, 0x0f,
	0xa3, 0x2b,
	0xa4, 0x0f,
	0xa5, 0x2b,
	0xa7, 0x9a,
	0xa8, 0x1c,
	0xa9, 0x11,
	0xaa, 0x16,
	0xab, 0x16,
	0xac, 0x3c,
	0xad, 0xf0,
	0xae, 0x57,
	0xc6, 0xaa,
	0xd2, 0x78,
	0xd0, 0xb4,
	0xd1, 0x00,
	0xc8, 0x10,
	0xc9, 0x12,
	0xd3, 0x09,
	0xd4, 0x2a,
	0xee, 0x4c,
	0x7e, 0xfa,
	0x74, 0xa7,
	0x78, 0x4e,
	0x60, 0xe7,
	0x61, 0xc8,
	0x6d, 0x70,
	0x1e, 0x39,
	0x98, 0x1a};

typedef struct
{
	int16_t flow_x_integral;	   // X 像素点累计时间内的累加位移(radians*10000)
								   // [除以 10000 乘以高度(mm)后为实际位移(mm)]
	int16_t flow_y_integral;	   // Y 像素点累计时间内的累加位移(radians*10000)
								   // [除以 10000 乘以高度(mm)后为实际位移(mm)]
	uint16_t integration_timespan; // 上一次发送光流数据到本次发送光流数据的累计时间（us）
	uint16_t ground_distance;	   // 预留。 默认为 999（0x03E7）
	uint8_t valid;				   // 状态值:0(0x00)为光流数据不可用
								   // 245(0xF5)为光流数据可用
	uint8_t version;			   // 版本号
} __PACKED _Flow;
static const unsigned char packet_ID[2] = {0xfe, 0x0a};

static void OpticalFlow_Server(void *pvParameters)
{
	DriverInfo driver_info = *(DriverInfo *)pvParameters;
	delete (DriverInfo *)pvParameters;

	/*发送初始化*/
	uint8_t buf[30];
	uint16_t len = sizeof(Sensor_cfg);

	// 0xAA指令
	buf[0] = 0xAA;
	driver_info.port.write(buf, 1, 1, 1);

	// 0xAB指令
	buf[0] = 0xAB;
	driver_info.port.write(buf, 1, 1, 1);
	driver_info.port.write(tab_focus, 4, 1, 1);
	buf[0] = tab_focus[0] ^ tab_focus[1] ^ tab_focus[2] ^ tab_focus[3];
	driver_info.port.write(buf, 1, 1, 1);
	os_delay(0.01);

	// 0xBB指令
	for (uint16_t i = 0; i < len; i += 2)
	{
		buf[0] = 0xBB;
		buf[1] = SENSOR_IIC_ADDR;
		buf[2] = Sensor_cfg[i];
		buf[3] = Sensor_cfg[i + 1];
		buf[4] = SENSOR_IIC_ADDR ^ Sensor_cfg[i] ^ Sensor_cfg[i + 1];
		driver_info.port.write(buf, 5, 1, 1);
	}
	os_delay(0.01);

	// 0xDD
	buf[0] = 0xDD;
	driver_info.port.write(buf, 1, 1, 1);
	/*发送初始化*/

	/*状态机*/
	_Flow Flow;
	unsigned char rc_counter = 0;
	signed char sum = 0;
	/*状态机*/

	while (1)
	{
		uint8_t rdata;
		if (driver_info.port.read(&rdata, 1, 2, 0.5))
		{
			if (rc_counter < 2)
			{
				// 接收包头
				if (rdata != packet_ID[rc_counter])
					rc_counter = 0;
				else
				{
					++rc_counter;
					sum = 0;
				}
			}
			else if (rc_counter < 12)
			{ // 接收数据
				((unsigned char *)&Flow)[rc_counter - 2] = rdata;
				sum ^= (signed char)rdata;
				++rc_counter;
			}
			else if (rc_counter == 12)
			{ // 校验
				if (sum != rdata)
					rc_counter = 0;
				else
					++rc_counter;
			}
			else
			{ // 接收包尾
				if (rdata == 0x55)
				{
					PosSensorHealthInf1 ZRange_inf;
					if (get_OptimalRange_Z(&ZRange_inf))
					{ // 测距传感器可用
						if (ZRange_inf.last_healthy_TIME.is_valid() && ZRange_inf.last_healthy_TIME.get_pass_time() < 50)
						{ // 测距50秒内健康
							// 获取高度
							double height = ZRange_inf.HOffset + ZRange_inf.PositionENU.z;
							// 获取角速度
							vector3<double> AngularRate;
							get_AngularRate_Ctrl(&AngularRate);
							// 补偿光流
							vector3<double> vel;
							double rotation_compensation_x = -constrain(AngularRate.y * 10000, 4500000000.0);
							double rotation_compensation_y = constrain(AngularRate.x * 10000, 4500000000.0);
							double integral_time = (Flow.integration_timespan * 1e-6f);
							if (integral_time > 1e-3)
							{
								double temp_flow_x, temp_flow_y;
								double flow_x, flow_y;
								temp_flow_x = Flow.flow_x_integral;
								temp_flow_y = -Flow.flow_y_integral;
								switch (driver_info.param)
								{
								case 0:
								default:
								{
									flow_x = temp_flow_x;
									flow_y = temp_flow_y;
									break;
								}
								case 1:
								{
									flow_x = temp_flow_y;
									flow_y = -temp_flow_x;
									break;
								}
								case 2:
								{
									flow_x = -temp_flow_x;
									flow_y = -temp_flow_y;
									break;
								}
								case 3:
								{
									flow_x = -temp_flow_y;
									flow_y = temp_flow_x;
									break;
								}
								}

								integral_time = 1.0 / integral_time;
								vel.x = (flow_x * integral_time - rotation_compensation_x) * 1e-4f * (1 + height);
								vel.y = (flow_y * integral_time - rotation_compensation_y) * 1e-4f * (1 + height);
								PositionSensorUpdateVel(default_optical_flow_index, driver_info.sensor_key, vel, true);
							}
							else
								PositionSensorSetInavailable(default_optical_flow_index, driver_info.sensor_key);
						}
						else
							PositionSensorSetInavailable(default_optical_flow_index, driver_info.sensor_key);
					}
					else
						PositionSensorSetInavailable(default_optical_flow_index, driver_info.sensor_key);
				}
				rc_counter = 0;
			}
		}
	}
}

static bool OpticalFlow_LC306_DriverInit(Port port, uint32_t param)
{
	// 波特率19200
	port.SetBaudRate(19200, 2, 2);
	// 注册传感器
	uint32_t sensor_key = PositionSensorRegister(default_optical_flow_index,
												 "LC306",
												 Position_Sensor_Type_RelativePositioning,
												 Position_Sensor_DataType_v_xy_nAC,
												 Position_Sensor_frame_BodyHeading,
												 0.1, 100);
	if (sensor_key == 0)
		return false;
	DriverInfo *driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	driver_info->sensor_key = sensor_key;
	xTaskCreate(OpticalFlow_Server, "OptFlowLC306", 1024, (void *)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_OpticalFlow_LC306()
{
	PortFunc_Register(33, OpticalFlow_LC306_DriverInit);
}