#include "Basic.hpp"
#include "drv_IICTFMini.hpp"
#include "drv_ExtIIC.hpp"
#include "SensorsBackend.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "MeasurementSystem.hpp"

#define SensorInd 2

struct DriverInfo
{
	uint32_t sensor_key;
};

typedef struct
{
	uint16_t header;
	uint16_t Dist;	   // Dist距离（30-1200cm）
	uint16_t Strength; // Strength信号强度（20-2000可信）
	uint16_t temp;
	uint8_t checksum;
} __PACKED _TfMini;

static void TFMini_Server(void *pvParameters)
{
	DriverInfo driver_info = *(DriverInfo *)pvParameters;
	delete (DriverInfo *)pvParameters;

	Aligned_DMABuf uint8_t tx_buf[32];
	Aligned_DMABuf uint8_t rx_buf[32];

	tx_buf[0] = 0x5a;
	tx_buf[1] = 0x05;
	tx_buf[2] = 0x00;
	tx_buf[3] = 0x01;
	tx_buf[4] = 0x60;

	bool res;
	_TfMini *data;
reTry:
	while (1)
	{
		ExtIIC_SendAddr7(0x10, tx_buf, 5);
		os_delay(0.1);

		res = ExtIIC_ReceiveAddr7(0x10, rx_buf, 9);
		if (res)
		{
			data = (_TfMini *)rx_buf;

			if (data->Strength > 20 && data->Dist > 1 && data->Dist < 5000)
			{
				vector3<double> position;
				position.z = data->Dist;
				// 获取倾角
				Quaternion quat;
				get_Airframe_quat(&quat);
				double lean_cosin = quat.get_lean_angle_cosin();
				// 更新
				position.z *= lean_cosin;
				PositionSensorUpdatePosition(SensorInd, driver_info.sensor_key, position, true);
			}
			else
				PositionSensorSetInavailable(SensorInd, driver_info.sensor_key);
		}
		else
			PositionSensorSetInavailable(SensorInd, driver_info.sensor_key);
	}
}

static bool I2C_TFMini_DriverInit()
{
	return true;
}
static bool I2C_TFMini_DriverRun()
{
	// 注册传感器
	uint32_t sensor_key = PositionSensorRegister(SensorInd,
												 "TFMini",
												 Position_Sensor_Type_RangePositioning,
												 Position_Sensor_DataType_s_z,
												 Position_Sensor_frame_ENU,
												 0.01, // 延时
												 0,	   // xy信任度
												 0	   // z信任度
	);
	if (sensor_key == 0)
		return false;
	DriverInfo *driver_info = new DriverInfo;
	driver_info->sensor_key = sensor_key;
	xTaskCreate(TFMini_Server, "TFMini", 1024, (void *)driver_info, SysPriority_ExtSensor, NULL);

	return true;
}

void init_drv_IICTFMini()
{
	I2CFunc_Register(33, I2C_TFMini_DriverInit, I2C_TFMini_DriverRun);
}
