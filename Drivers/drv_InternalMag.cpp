#include "Basic.hpp"
#include "drv_IntIIC.hpp"
#include "SensorsBackend.hpp"
#include "AC_Math.hpp"
#include "Commulink.hpp"

/*外置罗盘定义*/
	struct Internal_MagnetoMeter
	{
		//传感器名称
		SName name;
		
		//iic地址
		unsigned char device_address;
		//数据寄存器地址
		unsigned char data_address;
		
		//是否高位在前
		bool MSB;
		//是否需要手动开始采样
		bool need_set_sample;
		//采样寄存器地址
		unsigned char sample_address;
		//采样寄存器值
		unsigned char sample_cfg;
		
		//ID寄存器地址
		unsigned char ID_address;
		//ID
		unsigned char ID;
		
		//配置寄存器地址
		unsigned char configurations_addresses[10];
		//配置数目
		unsigned char configuration_count;
		//配置
		unsigned char configurations[10];
		//灵敏度（数据->Gauss）
		double sensitivity;
		
		//采样率（多少ms一次数据）
		unsigned char sample_mseconds;
		
		//轴向顺序
		//从1开始，负数为反向
		//如：1,-3,2代表轴向为 x , -z , y
		signed char axis_index[3];
		
		IICSpeed iic_speed;
	};
		
	static const Internal_MagnetoMeter Internal_MagnetoMeters[] = 
	{
	/*     名称     , iic地址, 数据寄存器地址 , 高位在前, 手动采样 , 采样寄存器地址, 采样设置, ID 地址 , ID   ,      配置寄存器地址      , 配置数目,  配置                                                     ,  灵敏度      , 采样率毫秒  , 轴向             ,  IIC速度                ,  */
		{ "QMC5883"   , 0x0d   ,      0         ,   false ,   false  ,          0  	 ,      0  ,   0x0d  , 0xff ,      { 0x9, 0xa, 0xb }   , 3       , { (0<<6) | (0<<4) | (0b10<<2) | (1<<0) , (1<<6) , 1 }     , 8.33333333e-5, 10          , { -2 ,  1 , 3 }  ,  IICSpeed_Standard100k    } , //QMC5883
    { "IST8310"   , 0x0c   ,      3         ,   false ,    true  ,          0xa  ,      1  ,      0  , 0x10 ,      { 0x41, 0x42 }      , 2       , { 0x24 , 0xc0 }                                           , 0.003        , 30          , { -2 , -1 , 3 }  ,  IICSpeed_Standard100k     } , //IST8310
		{ "FXOS8700"  , 0x1c   ,     0x33       ,   true ,    false  ,            0  ,      0  ,   0x0d  , 0xc7 ,      { 0x2a, 0x5b }      , 2       , { (3<<3)|(1<<0) , (7<<2)|(1<<0) }                         , 0.001        , 30          , { -2 , 1 , 3 }   ,  IICSpeed_Standard100k     } , //FXOS8700
		{ "QMC6983"   , 0x2c   ,      0         ,   false ,   false  ,            0  ,      0  ,   0x0c  , 0x01 ,  { 0x0b, 0x09, 0x0a }    , 3       , { 0xff, (0<<6)|(1<<4)|(3<<2)|(1<<0), (1<<6) }             , 0.0004       , 20          , { -1 , -2 , 3 }  ,  IICSpeed_Standard100k     } , //QMC6983
	};
	static const uint8_t Supported_Internal_MagnetoMeter_Count = sizeof( Internal_MagnetoMeters ) / sizeof( Internal_MagnetoMeter );
/*外置罗盘定义*/


static void InternalMag_Server(void* pvParameters)
{
ScanExtMag:
	//当前使用的外置罗盘型号
	int8_t current_ExtMag = -1;
	//缓冲区
	Aligned_DMABuf uint8_t tx_buf[32];
	Aligned_DMABuf uint8_t rx_buf[32];
	//前期采样延时
	uint16_t sample_delay = 0;
	//传感器key
	uint32_t key = 0;
	
	//操作结果
	bool res;
	//准确周期延时
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		for( int8_t i = 0; i < Supported_Internal_MagnetoMeter_Count; ++i )
		{
			vTaskDelay( 0.1*configTICK_RATE_HZ );
			const Internal_MagnetoMeter* sensor = &Internal_MagnetoMeters[i];
			
			//判断传感器ID是否正确
			tx_buf[0] = sensor->ID_address;
			res = IntIIC_SendReceiveAddr7( sensor->device_address, tx_buf, 1, rx_buf, 1, sensor->iic_speed );
			if(!res)
				continue;
			if( rx_buf[0] != sensor->ID )
				continue;
			
			//发送传感器配置			
			for( uint8_t k = 0; k < sensor->configuration_count; ++k )
			{
				tx_buf[0] = sensor->configurations_addresses[k];
				tx_buf[1] = sensor->configurations[k];
				res = IntIIC_SendAddr7( sensor->device_address, tx_buf, 2 );
				if(!res)
					break;
			}			
			if(!res)
				continue;
			
			//检查传感器配置是否成功
			for( uint8_t k = 0; k < sensor->configuration_count; ++k )
			{
				tx_buf[0] = sensor->configurations_addresses[k];
				res = IntIIC_SendReceiveAddr7( sensor->device_address, tx_buf, 1, rx_buf, 1 );
				if(!res)
					break;
				if( rx_buf[0] != sensor->configurations[k] )
				{	//配置校验错误
					res = false;
					break;
				}
			}
			if(!res)
				continue;
			
			//已成功识别到传感器
			key = IMUMagnetometerRegister( Internal_Magnetometer_Index,  SName("e")+SName(sensor->name) , sensor->sensitivity );
			if( key )
			{
				current_ExtMag = i;
				goto ExtMagDetected;
			}
			else
				os_delay(2);
		}
	}
	
ExtMagDetected:
	sample_delay = 0;
	const Internal_MagnetoMeter* sensor = &Internal_MagnetoMeters[current_ExtMag];
	while(1)
	{
		//周期对传感器采样
		vTaskDelay( sensor->sample_mseconds*1e-3*configTICK_RATE_HZ+1 );
		
		//采样
		uint8_t rt = 0;
		do
		{
			//失败次数过多重新扫描
			if( ++rt > 3 )
			{
				IMUMagnetometerUnRegister(Internal_Magnetometer_Index,key);
				goto ScanExtMag;
			}
			//读取数据
			tx_buf[0] = sensor->data_address;
			res = IntIIC_SendReceiveAddr7( sensor->device_address, tx_buf, 1, rx_buf, 6, sensor->iic_speed );
			if( sensor->need_set_sample )
			{	//需要发送采样指令
				tx_buf[0] = sensor->sample_address;
				tx_buf[1] = sensor->sample_cfg;
				res &= IntIIC_SendAddr7( sensor->device_address, tx_buf, 2 );
			}
		}while(res==false);
		
		/*更新传感器数据*/						
			//大端转小端
			if( sensor->MSB )
			{
				((uint16_t*)&rx_buf)[0] = __REV16( ((uint16_t*)&rx_buf)[0] );
				((uint16_t*)&rx_buf)[1] = __REV16( ((uint16_t*)&rx_buf)[1] );
				((uint16_t*)&rx_buf)[2] = __REV16( ((uint16_t*)&rx_buf)[2] );
			}			
			
			//轴向转换
			vector3<int32_t> data;
			if( sensor->axis_index[0] > 0 )
				data.x = (signed short)((uint16_t*)&rx_buf)[ sensor->axis_index[0] - 1 ];
			else
				data.x = -(signed short)((uint16_t*)&rx_buf)[ (-sensor->axis_index[0]) - 1 ];
			if( sensor->axis_index[1] > 0 )
				data.y = (signed short)((uint16_t*)&rx_buf)[ sensor->axis_index[1] - 1 ];
			else
				data.y = -(signed short)((uint16_t*)&rx_buf)[ (-sensor->axis_index[1]) - 1 ];
			if( sensor->axis_index[2] > 0 )
				data.z = (signed short)((uint16_t*)&rx_buf)[ sensor->axis_index[2] - 1 ];
			else
				data.z = -(signed short)((uint16_t*)&rx_buf)[ (-sensor->axis_index[2]) - 1 ];
				
			//更新数据
			if( sample_delay >= 50 )
				IMUMagnetometerUpdate( Internal_Magnetometer_Index,key, data, false);
			else
				++sample_delay;
		/*更新传感器数据*/
	}
}

static bool I2C_InternalMag_DriverInit()
{
	return true;
}
static bool I2C_InternalMag_DriverRun()
{
	xTaskCreate( InternalMag_Server, "intMag", 800, NULL, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_InternalMag()
{
	I2CFunc_Register( 8, I2C_InternalMag_DriverInit, I2C_InternalMag_DriverRun );
}
