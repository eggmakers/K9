#include "drv_US100.hpp"
#include "drv_ADM001.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

bool ADM001_last_read_ok = false;
uint16_t ADM001_recv_sum = 0;
uint16_t ADM001_calc_sum = 0;
float weight_kg = 0.0f;

struct DriverInfo
{
    uint32_t param;
    Port port;
    uint32_t sensor_key;
};

// 累加和校验函数
static uint8_t checksum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}

static void ADM001_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *)pvParameters;
    delete (DriverInfo *)pvParameters;
    const uint8_t read_cmd[4] = {0x01, 0x02, 0x00, 0x03}; // 读取重量指令
    uint8_t response[7];                                  // 预期响应：01 03 01 00 00 03 08
    driver_info.port.reset_rx(2);
    while (1)
    {
        // 发送读取指令
        driver_info.port.write(read_cmd, sizeof(read_cmd), 2, 0.5);

        // 读取响应数据
        bool read_ok = driver_info.port.read(response, sizeof(response), 2, 0.5);

        if (!read_ok)
        {
            driver_info.port.reset_rx(2);
            continue;
        }

        // 校验响应
        if (response[0] != 0x01 || response[1] != 0x03)
        {
            continue;
        }

        // 验证
        uint8_t calc_sum = checksum(response, sizeof(response) - 1); // 计算前6字节的和;
        uint8_t recv_sum = response[sizeof(response) - 1];           // 最后1字节是校验和
        ADM001_recv_sum = recv_sum;
        ADM001_calc_sum = calc_sum;
        if (calc_sum != recv_sum)
            continue;
        // 解析重量数据（小端序）
        uint32_t raw_weight = (response[3] << 16) | (response[4] << 8) | response[5];
        weight_kg = raw_weight / 1000.0f; // 转换为千克

        // 上报重量数据（假设使用Z轴表示重量）
        vector3<double> weight_data;
        weight_data.z = weight_kg;

        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms采样间隔
    }
}

static bool ADM001_DriverInit(Port port, uint32_t param)
{
    // 配置串口参数
    port.SetBaudRate(9600, 8, 1); // 8数据位，1停止位，无校验

    // 注册重量传感器
    uint32_t sensor_key = PositionSensorRegister(
        default_weight_sensor_index, // default_rtk_sensor_index
        "ADM001",
        Position_Sensor_Type_GlobalPositioning, // Position_Sensor_Type_GlobalPositioning
        Position_Sensor_DataType_s_z,           // Position_Sensor_DataType_s_z
        Position_Sensor_frame_ENU,              // Position_Sensor_frame_ENU
        0.1,                                    // 100ms延迟
        0.0,                                    // X不更新
        0.95                                    // Z轴高置信度
    );

    if (sensor_key == 0)
        return false;

    // 创建驱动任务
    DriverInfo *info = new DriverInfo;
    info->param = param;
    info->port = port;
    info->sensor_key = sensor_key;

    xTaskCreate(ADM001_Server,
                "ADM001",
                1536,
                info,
                SysPriority_ExtSensor,
                nullptr);

    return true;
}

void init_drv_ADM001()
{
    PortFunc_Register(54, ADM001_DriverInit); // 分配新端口号
}
