#include "Basic.hpp"
#include "FreeRTOS.h"
#include "drv_USB.hpp"
#include "drv_Uart1.hpp"
#include "drv_ADM001.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "mavlink.h"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"
#include "drv_PWMOUT.hpp"
#include "drv_US100.hpp"

float debug_test[30];

static void Debug_task(void *pvParameters)
{
	while (1)
	{
		uint8_t port_id = 0;
		const Port *port = get_CommuPort(port_id);
		if (port == 0 || port->write == 0)
		{
			os_delay(1);
			continue;
		}

		// MAVLink消息打包
		mavlink_message_t msg_sd;

		if (mavlink_lock_chan(port_id, 0.01))
		{
			Position_Sensor_Data weight_data;
			if (GetPositionSensorData(default_weight_sensor_index, &weight_data))
			{
				// 根据ADM001协议解析数据
				float real_weight = weight_data.position.z; // Z轴存储重量
				uint16_t raw_value = ADM001_recv_sum;		// X轴原始值
				float temp = (float)ADM001_calc_sum;

				mavlink_msg_debug_vect_pack_chan(
					get_CommulinkSysId(),
					get_CommulinkCompId(), // 组件ID
					port_id,
					&msg_sd,
					"ADM_W",						   // 自定义标识符
					TIME::get_System_Run_Time() * 1e6, // 使用传感器时间戳
					real_weight,					   // X: 实际重量(kg)
					raw_value,						   // Y: 原始AD值
					temp							   // Z: 温度(℃)
				);
				mavlink_msg_to_send_buffer(
					port->write,
					port->lock,
					port->unlock,
					&msg_sd,
					0, // 优先级
					-1 // 无超时
				);
			}
			mavlink_unlock_chan(port_id);
		}

		os_delay(0.05); // 保持20Hz频率
	}
}

void init_Debug()
{
	xTaskCreate(Debug_task, "Debug", 1500, NULL, SysPriority_UserTask, NULL);
}