#include "drv_Main.hpp"

#include "SensorsBackend.hpp"
#include "drv_LED.hpp"
#include "drv_USB.hpp"
#include "drv_Uart1.hpp"
#include "drv_Uart3.hpp"
#include "drv_Uart5.hpp"
#include "drv_Uart7.hpp"
#include "drv_Uart8.hpp"
#include "Commulink.hpp"
#include "drv_SDMMC.hpp"
#include "drv_Sensors.hpp"
#include "drv_Flash.hpp"
#include "drv_RCSbus.hpp"
#include "drv_RCPPM.hpp"
#include "drv_PWMOut.hpp"
#include "drv_ADC.hpp"
#include "drv_CRC.hpp"

#include "StorageSystem.hpp"
#include "Parameters.hpp"
#include "drv_BootLoader.hpp"

// 串口设备
#include "drv_CRSF.hpp"
// GPS
#include "drv_GPS.hpp"
#include "drv_GPS_nmea.hpp"
#include "drv_RTK.hpp"
#include "drv_RTK_Track.hpp"
#include "drv_RTK_DAO_Base.hpp"
#include "drv_RTK_DAO_Move.hpp"
#include "drv_mosaxicX5.hpp"
#include "drv_mosaxicX6.hpp"
#include "drv_UM482.hpp"
#include "drv_UM982.hpp"
#include "drv_UM982_movingTrack.hpp"
#include "drv_RTCMInput.hpp"
// 光流
#include "drv_OpticalFlow_LC302.hpp"
#include "drv_OpticalFlow_LC306.hpp"
#include "drv_OpticalFlow_JL32xx.hpp"
#include "drv_OpticalFlow_GL9306.hpp"
// 串口定高
#include "drv_TFMini.hpp"
#include "drv_ISTRA24.hpp"
#include "drv_ISTRA24_2.hpp"
#include "drv_NRA24.hpp"
#include "drv_US100.hpp"
// 串口定位
#include "drv_UWB_LinkTrack.hpp"
// 串口云台
#include "drv_YT_ZR10.hpp"
#include "drv_YT_INYYO.hpp"
#include "drv_YT_RYEOP.hpp"
#include "drv_YT_Viewlink.hpp"
// 串口避障
#include "drv_TFMiniAv.hpp"
#include "drv_MR72_UartAv.hpp"
// 二次开发接口
#include "drv_SDI.hpp"
#include "drv_ADM001.hpp"

// CAN设备
#include "drv_Can.hpp"
#include "drv_CAN_BMS_TATTU.hpp"
#include "drv_CAN_BMS_ZHIAN.hpp"
// CAN定高
#include "drv_CAN_Radar_TR60_AH.hpp"
// CAN避障
#include "drv_CAN_QOAR1271_Av.hpp"
#include "drv_CAN_MR72_Av.hpp"
#include "drv_CAN_MR72_BackAv.hpp"
// CAN发动机
#include "drv_CAN_Engine_Zongshen.hpp"
#include "drv_CAN_Engine_Xuanfu_16KW.hpp"
#include "drv_CAN_Engine_Xuanfu_7KW.hpp"
// IIC设备
#include "drv_IntIIC.hpp"
#include "drv_ExtIIC.hpp"
#include "drv_InternalMag.hpp"
#include "drv_ExtMag.hpp"
#include "drv_ExtLed_TCA62724FMG.hpp"
#include "drv_IIC_IRLock.hpp"
// IIC定高
#include "drv_ExtBarometer.hpp"
#include "drv_IICTFMini.hpp"
// IIC避障
#include "drv_IICTFMini_Av.hpp"

void init_drv_Main()
{
	// LED初始化
	init_drv_LED();
	// ADC初始化
	init_drv_ADC();
	// 等待电压稳定
	while (adcGet_VDDA_Voltage() < 3.0)
	{ // 电压过低不初始化flash防止误擦除
		os_delay(0.1);
	}
	init_drv_CRC();

	// 内部存储介质初始化
	init_drv_Flash();
	init_InternalStorage();
	// 参数表初始化
	init_Parameters();

	// 存储外设驱动
	init_drv_SDMMC();
	// BL更新检测
	init_drv_BootLoader();

	// 传感器接口初始化
	init_Sensors();
	init_Commulink();
	os_delay(0.1);

	init_drv_RCPPM();
	// 初始化PWM输出
	init_drv_PWMOut();

	// SD存储初始化
	init_SDStorage();

	// 通信端口初始化
	init_drv_USB();
	init_drv_Uart1();
	init_drv_Uart3();
	init_drv_Uart5();
	init_drv_Uart7();
	init_drv_Uart8();

	// 内部传感器驱动
	init_drv_Sensors();
	init_drv_IntIIC();
	// 接收机驱动
	init_drv_RCSbus();
	init_drv_CRSF();

	// 外置IIC驱动
	init_drv_ExtIIC();
	init_drv_InternalMag();
	init_drv_ExtMag();
	init_drv_ExtLed_TCA62724FMG();
	init_drv_IICIRLock();
	// IIC定高
	init_drv_ExtBarometer();
	init_drv_IICTFMini();
	// IIC避障
	init_drv_IICTFMiniAv();

	// 串口驱动
	// GPS
	init_drv_GPS();
	init_drv_GPS_nmea();
	init_drv_RTK();
	init_drv_RTK_Track();
	init_drv_RTK_DAO_Base();
	init_drv_RTK_DAO_Move();
	init_drv_RTK_MosaxicX5();
	init_drv_RTK_MosaxicX6();
	init_drv_RTK_UM482();
	init_drv_RTK_UM982();
	init_drv_RTK_UM982_movingTrack();
	init_drv_RTCMInput();
	// 光流
	init_drv_OpticalFlow_LC302();
	init_drv_OpticalFlow_LC306();
	init_drv_OpticalFlow_JL32xx();
	init_drv_OpticalFlow_GL9306();
	// 串口定高
	init_drv_TFMini();
	init_drv_ISTRA24();
	init_drv_ISTRA24_2();
	init_drv_NRA24();
	init_drv_US100();
	// 串口定位
	init_drv_UWB_LinkTrack();
	// 串口云台
	init_drv_YT_ZR10();
	init_drv_YT_INYYO();
	init_drv_YT_RYEOP();
	init_drv_YT_Viewlink();
	// 串口避障
	init_drv_TFMiniAv();
	init_drv_MR72_UartAv();
	// 二次开发接口
	// init_drv_SDI();
	init_drv_ADM001();

	// CAN驱动
	init_drv_CAN_BMS_TATTU();
	init_drv_CAN_BMS_ZHIAN();
	// CAN定高
	init_drv_Radar_TR60_AH();
	// CAN避障
	init_drv_CAN_QOAR1271_Av();
	init_drv_CAN_MR72_Av();
	init_drv_CAN_MR72_BackAv();
	// CAN发动机
	init_drv_Engine_Zongshen();
	init_drv_Engine_Xuanfu_16KW();
	init_drv_Engine_Xuanfu_7KW();
}