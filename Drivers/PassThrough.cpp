#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

/* 透传传感器数据结构 */
struct PassThroughDriverInfo
{
    uint32_t param;
    Port port;
    uint32_t sensor_key;
};

/* 透传数据格式 */
typedef struct
{
    uint8_t data[32]; // 最大32字节透传数据
    uint8_t length;   // 数据长度
} __PACKED PassThroughData;

/* 透传数据处理任务 */
static void PassThrough_Server(void *pvParameters)
{
    PassThroughDriverInfo driver_info = *(PassThroughDriverInfo *)pvParameters;
    delete (PassThroughDriverInfo *)pvParameters;

    PassThroughData pass_data;
    while (1)
    {
        uint8_t rdata;
        if (driver_info.port.read(&rdata, 1, 2, 0.5)) // 读取1字节数据
        {
            static uint8_t buffer[32];
            static uint8_t index = 0;

            buffer[index++] = rdata;

            if (index >= sizeof(buffer))
            {
                // 存入传感器
                vector3<double> data_vector;
                memcpy(&data_vector, buffer, sizeof(vector3<double>));

                PassThroughUpdate(driver_info.sensor_key, data_vector, true);
                index = 0; // 重置索引
            }
        }
    }
}

/* 透传传感器驱动初始化 */
static bool PassThrough_DriverInit(Port port, uint32_t param)
{
    port.SetBaudRate(115200, 2, 2); // 设置波特率

    uint32_t sensor_key = PassThroughRegister(0, "PassThrough", 100);
    if (sensor_key == 0)
        return false;

    PassThroughDriverInfo *driver_info = new PassThroughDriverInfo;
    driver_info->param = param;
    driver_info->port = port;
    driver_info->sensor_key = sensor_key;

    xTaskCreate(PassThrough_Server, "PassThroughSrv", 1024, (void *)driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

/* 透传传感器注册 */
void init_drv_PassThrough()
{
    PortFunc_Register(99, PassThrough_DriverInit);
}
