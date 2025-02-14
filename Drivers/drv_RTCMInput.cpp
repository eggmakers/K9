#include "drv_RTCMInput.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

#define SensorInd 2

struct DriverInfo
{
	uint32_t param;
	Port port;
};

static void RTCMInput_Server(void *pvParameters)
{
	DriverInfo driver_info = *(DriverInfo *)pvParameters;
	delete (DriverInfo *)pvParameters;

// 状态机
#define BUFFER_LEN 1024
	uint8_t _buffer[BUFFER_LEN + 2];
	uint16_t _pos = 0;
	uint8_t parse_state = 0;
	uint16_t _message_length = 0;

	while (1)
	{
		uint8_t rdata;
		if (driver_info.port.read(&rdata, 1, 2, 0.5))
		{
			_buffer[_pos++] = rdata;
			if (parse_state == 0)
			{
				if (rdata == 0xD3)
					parse_state = 1;
				else
				{
					_pos = 0;
					_message_length = 0;
				}
			}
			else
			{
				if (_pos == 3)
				{
					_message_length = static_cast<uint16_t>((((_buffer[1] & 3) << 8) | (_buffer[2])));
					if (_message_length + 6 > BUFFER_LEN)
					{
						parse_state = 0;
						_pos = 0;
					}
				}

				if (_pos >= _message_length + 6)
				{ // 接收完成
					inject_RtkPorts(_buffer, _pos);
					_pos = 0;
					parse_state = 0;
				}
			}
		}
	}
}

static bool RTCMInput_DriverInit(Port port, uint32_t param)
{
	// 波特率115200
	if (param < 1000)
		port.SetBaudRate(115200, 2, 2);
	else
		port.SetBaudRate(param, 2, 2);

	DriverInfo *driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate(RTCMInput_Server, "RTCMIn", 1024, (void *)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_RTCMInput()
{
	PortFunc_Register(8, RTCMInput_DriverInit);
}