#include "Basic.hpp"
#include "drv_Sensors.hpp"

#include "FreeRTOS.h"
#include "timers.h"
#include "SensorsBackend.hpp"
#include "AC_Math.hpp"
#include "MS_Main.hpp"
#include "TD4.hpp"
#include <limits>
#include "MeasurementSystem.hpp"

/*DMA操作*/
#define DMATxStream_Enable_MINC (DMA1_Stream0->CR |= (1 << 10))
#define DMATxStream_Disable_MINC (DMA1_Stream0->CR &= ~(1 << 10))

#define DMARxStream_Enable_MINC (DMA1_Stream1->CR |= (1 << 10))
#define DMARxStream_Disable_MINC (DMA1_Stream1->CR &= ~(1 << 10))

#define DMARxStream_Enable_IRQ (NVIC_EnableIRQ(DMA1_Stream1_IRQn))
#define DMARxStream_Disable_IRQ (NVIC_DisableIRQ(DMA1_Stream1_IRQn))
/*DMA操作*/

/*传感器CS引脚操作*/
#define PullUp_BMI088Gyro_CS (GPIOE->BSRR = (1 << 10))
#define PullUp_BMI088Accel_CS (GPIOE->BSRR = (1 << 9))
#define PullUp_SPL06_CS (GPIOC->BSRR = (1 << 4))
#define PullUp_ICM20689_CS (GPIOE->BSRR = (1 << 11))

#define PullDown_BMI088Gyro_CS (GPIOE->BSRR = (1 << 26))
#define PullDown_BMI088Accel_CS (GPIOE->BSRR = (1 << 25))
#define PullDown_SPL06_CS (GPIOC->BSRR = (1 << 20))
#define PullDown_ICM20689_CS (GPIOE->BSRR = (1 << 27))
/*传感器CS引脚操作*/

/*SPI操作*/
// 在SPI发送数据
static void SPI1_Transmit_Start(const uint8_t *TxBuf, uint16_t size);
// 在SPI发送数据并等待发送完成
static void SPI1_Transmit(const uint8_t *TxBuf, uint16_t size);

// 在SPI接收数据
static void SPI1_TransmitReceive_Start(const uint8_t *TxBuf, uint8_t *RxBuf, uint16_t size);
// 在SPI接收数据并等待接收完成
static void SPI1_TransmitReceive(const uint8_t *TxBuf, uint8_t *RxBuf, uint16_t size);
/*SPI操作*/

/*传感器状态机*/

#ifdef DCACHE_SIZE
#define TxBufferSize DCACHE_SIZE
#define RxBufferSize DCACHE_SIZE
static Aligned_DMABuf uint8_t TxBuf[TxBufferSize];
static Aligned_DMABuf uint8_t RxBuf[RxBufferSize];
#else
#define BufferSize 32
static uint8_t TxBuf[BufferSize];
static uint8_t RxBuf[BufferSize];
#endif
static int8_t current_read_sensor = -1; //-1:无传感器
										// 0:BMI088_Gyro
										// 1:BMI088_Accel
										// 8:AK8975读阶段
										// 9:AK8975写阶段
										// 16:SPL06
										// 20:icm20689
static bool nd_read_bmi088Gyro = false;
static bool nd_read_bmi088Accel = false;
static bool nd_read_icm20689 = false;
static bool nd_read_AK8975 = false;
static bool nd_read_SPL06 = false;

#define TemperatureSensors_Count 3
static float IMU_Temperature[TemperatureSensors_Count] = {0};
float internal_barometer_temperature = 20;
float internal_barometer_pressure = 101325.01576;
float internal_barometer_altitude = 0;

static int8_t bmi088Ind = -1;
static int8_t icm20689Ind = -1;
static uint8_t imuIndCounter = 0;
/*传感器状态机*/

/*BMI088数据*/
// 陀螺
struct __BMI088_GyroData
{
	uint8_t rsv1;
	int16_t gyro[3]; // 0x02-0x07
} __PACKED;

// 加速度
struct __BMI088_AccelData
{
	uint8_t rsv1[2];
	int16_t acc[3];			   // 0x12-0x17
	uint32_t sensor_time : 24; // 0x18-0x1a
	uint8_t rsv2[2];		   // 0x1b-0x1c
	uint8_t acc_int_stat;	   // 0x1d
	uint8_t rsv3[4];		   // 0x1e-0x21
	uint16_t temperature;	   // 0x22-0x23
} __PACKED;
/*BMI088数据*/

/*ICM20689数据*/
struct __ICM20689_Data
{
	uint8_t rsv1;
	int16_t acc[3];
	int16_t temperature;
	int16_t gyro[3];
} __PACKED;
/*ICM20689数据*/

/*AK8975校准数据*/
static vector3<double> AK8974_ASA;
struct __AK8975_Data
{
	uint8_t rsv1;
	int16_t mag[3]; // 0x03-0x08
} __PACKED;
/*AK8975校准数据*/

/*SPL06校准数据*/
static uint32_t internal_barometer_key = 0;
struct SPL06_COEFFICIENTS
{
	int16_t c0;
	int16_t c1;
	int32_t c00;
	int32_t c10;
	int16_t c01;
	int16_t c11;
	int16_t c20;
	int16_t c21;
	int16_t c30;
	double KP;
	double KT;
};
static SPL06_COEFFICIENTS SPL06_coefficients;
struct __SPL06_Data
{
	uint8_t rsv1;
	unsigned int pressure : 24;
	unsigned int temperature : 24;
} __PACKED;

static void spl06_pressure_rateset(uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
	uint8_t reg = 0;
	switch (u8SmplRate)
	{
	case 2:
		reg |= (1 << 4);
		break;
	case 4:
		reg |= (2 << 4);
		break;
	case 8:
		reg |= (3 << 4);
		break;
	case 16:
		reg |= (4 << 4);
		break;
	case 32:
		reg |= (5 << 4);
		break;
	case 64:
		reg |= (6 << 4);
		break;
	case 128:
		reg |= (7 << 4);
		break;
	case 1:
	default:
		break;
	}
	switch (u8OverSmpl)
	{
	case 2:
		reg |= 1;
		SPL06_coefficients.KP = 1.0 / 1572864;
		break;
	case 4:
		reg |= 2;
		SPL06_coefficients.KP = 1.0 / 3670016;
		break;
	case 8:
		reg |= 3;
		SPL06_coefficients.KP = 1.0 / 7864320;
		break;
	case 16:
		SPL06_coefficients.KP = 1.0 / 253952;
		reg |= 4;
		break;
	case 32:
		SPL06_coefficients.KP = 1.0 / 516096;
		reg |= 5;
		break;
	case 64:
		SPL06_coefficients.KP = 1.0 / 1040384;
		reg |= 6;
		break;
	case 128:
		SPL06_coefficients.KP = 1.0 / 2088960;
		reg |= 7;
		break;
	case 1:
	default:
		SPL06_coefficients.KP = 1.0 / 524288;
		break;
	}
	TxBuf[0] = (0 << 7) | 0x06;
	TxBuf[1] = reg;
	PullDown_SPL06_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_SPL06_CS;
	os_delay(0.01);
	if (u8OverSmpl > 8)
	{
		TxBuf[0] = (1 << 7) | 0x09;
		PullDown_SPL06_CS;
		SPI1_TransmitReceive(TxBuf, RxBuf, 2);
		PullUp_SPL06_CS;
		os_delay(0.01);

		TxBuf[0] = (0 << 7) | 0x09;
		TxBuf[1] = RxBuf[1] | (1 << 2);
		PullDown_SPL06_CS;
		SPI1_Transmit(TxBuf, 2);
		PullUp_SPL06_CS;
		os_delay(0.01);
	}
}
static void spl06_temperature_rateset(uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
	uint8_t reg = 0;
	switch (u8SmplRate)
	{
	case 2:
		reg |= (1 << 4);
		break;
	case 4:
		reg |= (2 << 4);
		break;
	case 8:
		reg |= (3 << 4);
		break;
	case 16:
		reg |= (4 << 4);
		break;
	case 32:
		reg |= (5 << 4);
		break;
	case 64:
		reg |= (6 << 4);
		break;
	case 128:
		reg |= (7 << 4);
		break;
	case 1:
	default:
		break;
	}
	switch (u8OverSmpl)
	{
	case 2:
		reg |= 1;
		SPL06_coefficients.KT = 1.0f / 1572864;
		break;
	case 4:
		reg |= 2;
		SPL06_coefficients.KT = 1.0f / 3670016;
		break;
	case 8:
		reg |= 3;
		SPL06_coefficients.KT = 1.0f / 7864320;
		break;
	case 16:
		SPL06_coefficients.KT = 1.0f / 253952;
		reg |= 4;
		break;
	case 32:
		SPL06_coefficients.KT = 1.0f / 516096;
		reg |= 5;
		break;
	case 64:
		SPL06_coefficients.KT = 1.0f / 1040384;
		reg |= 6;
		break;
	case 128:
		SPL06_coefficients.KT = 1.0f / 2088960;
		reg |= 7;
		break;
	case 1:
	default:
		SPL06_coefficients.KT = 1.0f / 524288;
		break;
	}
	TxBuf[0] = (0 << 7) | 0x07;
	TxBuf[1] = (1 << 7) | reg;
	PullDown_SPL06_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_SPL06_CS;
	os_delay(0.01);
	if (u8OverSmpl > 8)
	{
		TxBuf[0] = (1 << 7) | 0x09;
		PullDown_SPL06_CS;
		SPI1_TransmitReceive(TxBuf, RxBuf, 2);
		PullUp_SPL06_CS;
		os_delay(0.01);

		TxBuf[0] = (0 << 7) | 0x09;
		TxBuf[1] = RxBuf[1] | (1 << 2);
		PullDown_SPL06_CS;
		SPI1_Transmit(TxBuf, 2);
		PullUp_SPL06_CS;
		os_delay(0.01);
	}
}
/*SPL06校准数据*/

/*BMI088初始化*/
static uint32_t bmi088_gyro_key = 0;
static uint32_t bmi088_accel_key = 0;
static bool BMI088_Init()
{
	/*加速度计初始化*/
	// 将加速度计接口设置为SPI
	TxBuf[0] = (1 << 7) | 0;
	PullDown_BMI088Accel_CS;
	SPI1_TransmitReceive(TxBuf, RxBuf, 3);
	PullUp_BMI088Accel_CS;
	os_delay(0.1);

	// 检验Chip ID
	TxBuf[0] = (1 << 7) | 0;
	PullDown_BMI088Accel_CS;
	SPI1_TransmitReceive(TxBuf, RxBuf, 3);
	PullUp_BMI088Accel_CS;
	os_delay(0.01);
	if (RxBuf[2] != 0x1e)
		return false;
	os_delay(0.01);

	// 复位
	TxBuf[0] = (0 << 7) | 0x7e;
	TxBuf[1] = 0xb6;
	PullDown_BMI088Accel_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_BMI088Accel_CS;
	os_delay(0.1);

	// 将加速度计接口设置为SPI
	TxBuf[0] = (1 << 7) | 0;
	PullDown_BMI088Accel_CS;
	SPI1_TransmitReceive(TxBuf, RxBuf, 3);
	PullUp_BMI088Accel_CS;
	os_delay(0.1);

	// 打开加速度计
	TxBuf[0] = (0 << 7) | 0x7d;
	TxBuf[1] = 0x04;
	PullDown_BMI088Accel_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_BMI088Accel_CS;
	os_delay(0.1);

	// 加速度计ODR 800hz
	TxBuf[0] = (0 << 7) | 0x40;
	TxBuf[1] = (0x8 << 4) | (0xB << 0);
	PullDown_BMI088Accel_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_BMI088Accel_CS;
	os_delay(0.01);

	// 加速度计量程24g
	TxBuf[0] = (0 << 7) | 0x41;
	TxBuf[1] = 0x03;
	PullDown_BMI088Accel_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_BMI088Accel_CS;
	os_delay(0.01);

	// 进入Active模式
	TxBuf[0] = (0 << 7) | 0x7c;
	TxBuf[1] = 0x00;
	PullDown_BMI088Accel_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_BMI088Accel_CS;
	os_delay(0.1);

	bmi088Ind = imuIndCounter++;
	set_IMU_Accelerometer_UpdateFreq(bmi088Ind, 1000);
	bmi088_accel_key = IMUAccelerometerRegister(bmi088Ind, "BMI088", GravityAcc / 1365.0, 0.1);
	/*加速度计初始化*/

	/*陀螺初始化*/
	// 检验Chip ID
	TxBuf[0] = (1 << 7) | 0;
	PullDown_BMI088Gyro_CS;
	SPI1_TransmitReceive(TxBuf, RxBuf, 2);
	PullUp_BMI088Gyro_CS;
	if (RxBuf[1] != 0x0f)
		while (1)
			;
	os_delay(0.01);

	// 复位
	TxBuf[0] = (0 << 7) | 0x14;
	TxBuf[1] = 0xb6;
	PullDown_BMI088Gyro_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_BMI088Gyro_CS;
	os_delay(0.1);

	// GYRO_LPM1(0x11): normal mode
	TxBuf[0] = (0 << 7) | 0x11;
	TxBuf[1] = 0x00;
	PullDown_BMI088Gyro_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_BMI088Gyro_CS;
	os_delay(0.1);

	// GYRO_RANGE(0x0f): 2000deg/s
	TxBuf[0] = (0 << 7) | 0x0f;
	TxBuf[1] = 0x00;
	PullDown_BMI088Gyro_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_BMI088Gyro_CS;
	os_delay(0.01);

	// GYRO_BANDWIDTH(0x10): ODR=2000hz bandwidth=230hz
	TxBuf[0] = (0 << 7) | 0x10;
	TxBuf[1] = 0x02;
	PullDown_BMI088Gyro_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_BMI088Gyro_CS;
	os_delay(0.01);

	bmi088_gyro_key = IMUGyroscopeRegister(bmi088Ind, "BMI088", 0.00106526443603169529841533860381, 0.1);
	set_IMU_Gyroscope_UpdateFreq(bmi088Ind, 2000);
	/*陀螺初始化*/
	return true;
}
/*BMI088初始化*/

/*ICM20689初始化*/
static uint32_t icm20689_gyro_key = 0;
static uint32_t icm20689_accel_key = 0;
static bool ICM20689_Init()
{
	// 校验Chip ID
	TxBuf[0] = (1 << 7) | 117;
	PullDown_ICM20689_CS;
	SPI1_TransmitReceive(TxBuf, RxBuf, 2);
	PullUp_ICM20689_CS;
	os_delay(0.01);
	if (RxBuf[1] != 0x98)
		return false;

	// 复位
	TxBuf[0] = (0 << 7) | 107;
	TxBuf[1] = (1 << 7);
	PullDown_ICM20689_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_ICM20689_CS;
	os_delay(0.3);

	// 设置 ICM20689 时钟源，Auto selects the best available clock source
	TxBuf[0] = (0 << 7) | 107;
	TxBuf[1] = (1 << 0);
	PullDown_ICM20689_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_ICM20689_CS;
	os_delay(0.1);

	// 设置 ICM20689 陀螺量程:±2000dps
	TxBuf[0] = (0 << 7) | 27;
	TxBuf[1] = (3 << 3);
	PullDown_ICM20689_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_ICM20689_CS;
	os_delay(0.01);

	// 设置 ICM20689  陀螺DLPF： 250hz
	TxBuf[0] = (0 << 7) | 26;
	TxBuf[1] = 0;
	PullDown_ICM20689_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_ICM20689_CS;
	os_delay(0.01);

	// 设置 ICM20689 加计量程: ± 16G
	TxBuf[0] = (0 << 7) | 28;
	TxBuf[1] = (3 << 3);
	PullDown_ICM20689_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_ICM20689_CS;
	os_delay(0.01);

	// 设置 ICM20689 加速度计DLPF： 1046.0  hz
	TxBuf[0] = (0 << 7) | 29;
	TxBuf[1] = (0 << 3) | (0 << 0);
	PullDown_ICM20689_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_ICM20689_CS;
	os_delay(0.01);

	// 使能 ICM20689 加计和陀螺
	TxBuf[0] = (0 << 7) | 107;
	TxBuf[1] = 0;
	PullDown_ICM20689_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_ICM20689_CS;
	os_delay(0.01);

	// 注册陀螺仪
	icm20689Ind = imuIndCounter++;
	icm20689_gyro_key = IMUGyroscopeRegister(icm20689Ind, "ICM20689", 0.00106526443603169529841533860381, 0.1);
	set_IMU_Gyroscope_UpdateFreq(icm20689Ind, 1000);

	// 注册加速度计
	set_IMU_Accelerometer_UpdateFreq(icm20689Ind, 1000);
	icm20689_accel_key = IMUAccelerometerRegister(icm20689Ind, "ICM20689", GravityAcc / 2048.0, 0.1);
	/*ICM20649初始化*/
	return true;
}
/*ICM20689初始化*/

/*SPL06初始化*/
static bool SPL06_Init()
{
	// 复位
	TxBuf[0] = (0 << 7) | 0x0c;
	TxBuf[1] = (1 << 7) | (0b1001);
	PullDown_SPL06_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_SPL06_CS;
	os_delay(0.1);

	// 配置传感器
	spl06_pressure_rateset(64, 32);	   // pressure 64 samples per sec , 32 times over sampling
	spl06_temperature_rateset(128, 2); // temperature 128 samples per sec , 2 times over sampling
	TxBuf[0] = (0 << 7) | 0x08;
	TxBuf[1] = 0b111;
	PullDown_SPL06_CS;
	SPI1_Transmit(TxBuf, 2);
	PullUp_SPL06_CS;
	os_delay(0.1);

	TxBuf[0] = (1 << 7) | 0x10;
	PullDown_SPL06_CS;
	SPI1_TransmitReceive(TxBuf, RxBuf, 19);
	PullUp_SPL06_CS;
	SPL06_coefficients.c0 = (RxBuf[1] << 4) | (RxBuf[2] >> 4);
	SPL06_coefficients.c0 = (SPL06_coefficients.c0 & 0x0800) ? (0xF000 | SPL06_coefficients.c0) : SPL06_coefficients.c0;
	SPL06_coefficients.c1 = ((RxBuf[2] & 0xf) << 8) | (RxBuf[3]);
	SPL06_coefficients.c1 = (SPL06_coefficients.c1 & 0x0800) ? (0xF000 | SPL06_coefficients.c1) : SPL06_coefficients.c1;
	SPL06_coefficients.c00 = (RxBuf[4] << 12) | (RxBuf[5] << 4) | (RxBuf[6] >> 4);
	SPL06_coefficients.c00 = (SPL06_coefficients.c00 & 0x080000) ? (0xFFF00000 | SPL06_coefficients.c00) : SPL06_coefficients.c00;
	SPL06_coefficients.c10 = ((RxBuf[6] & 0xf) << 16) | (RxBuf[7] << 8) | (RxBuf[8] >> 0);
	SPL06_coefficients.c10 = (SPL06_coefficients.c10 & 0x080000) ? (0xFFF00000 | SPL06_coefficients.c10) : SPL06_coefficients.c10;
	SPL06_coefficients.c01 = (RxBuf[9] << 8) | (RxBuf[10] << 0);
	SPL06_coefficients.c11 = (RxBuf[11] << 8) | (RxBuf[12] << 0);
	SPL06_coefficients.c20 = (RxBuf[13] << 8) | (RxBuf[14] << 0);
	SPL06_coefficients.c21 = (RxBuf[15] << 8) | (RxBuf[16] << 0);
	SPL06_coefficients.c30 = (RxBuf[17] << 8) | (RxBuf[18] << 0);
	// 注册传感器
	internal_barometer_key = PositionSensorRegister(internal_baro_sensor_index,
													"SPL06",
													Position_Sensor_Type_RelativePositioning,
													Position_Sensor_DataType_s_z,
													Position_Sensor_frame_ENU,
													0.05, // 延时
													0,	  // xy信任度
													50	  // z信任度
	);
	return true;
}
/*SPL06初始化*/
void init_drv_Sensors()
{
	/*IO初始化
		SPI1_SCK:PA5
		SPI1_MOSI:PA7
		SPI1_MISO:PA6

		BMI088_Gyro_Cs:PE10
		BMI088_Accel_Cs:PE9
	  ICM20689_CS:PE11
		SPL06_Cs:PC4
		AK8975_Cs:PA4
	*/
	// 打开GPIO时钟
	RCC->AHB4ENR |= (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0);
	delay(1e-5);

	// 设置Moder复用功能（SPI）、输出模式（CS）
	set_register(GPIOA->MODER, 0b10, 10, 2); // PA5
	set_register(GPIOA->MODER, 0b10, 14, 2); // PA7
	set_register(GPIOA->MODER, 0b10, 12, 2); // PA6
	set_register(GPIOE->MODER, 0b01, 22, 2); // PE11
	set_register(GPIOE->MODER, 0b01, 20, 2); // PE10
	set_register(GPIOE->MODER, 0b01, 18, 2); // PE9
	set_register(GPIOC->MODER, 0b01, 8, 2);	 // PC4

	// SCLK、MOSI、CS推挽输出，MISO开漏上拉
	GPIOA->OTYPER |= (1 << 6);
	set_register(GPIOA->PUPDR, 0b01, 12, 2); // PA6

	// 设置速度
	set_register(GPIOA->OSPEEDR, 0b01, 10, 2); // PA5
	set_register(GPIOA->OSPEEDR, 0b01, 14, 2); // PA7
	set_register(GPIOA->OSPEEDR, 0b01, 12, 2); // PA6
	set_register(GPIOE->OSPEEDR, 0b01, 22, 2); // PE11
	set_register(GPIOE->OSPEEDR, 0b01, 20, 2); // PE10
	set_register(GPIOE->OSPEEDR, 0b01, 18, 2); // PE9
	set_register(GPIOC->OSPEEDR, 0b01, 8, 2);  // PC4

	// 设置复用功能6
	set_register(GPIOA->AFR[0], 5, 20, 4); // PA5
	set_register(GPIOA->AFR[0], 5, 28, 4); // PA7
	set_register(GPIOA->AFR[0], 5, 24, 4); // PA6
	/*IO初始化*/

	/*SPI1初始化*/
	// 拉高所有CS
	PullUp_BMI088Gyro_CS;
	PullUp_BMI088Accel_CS;
	PullUp_ICM20689_CS;
	PullUp_SPL06_CS;

	// 打开SPI1时钟
	RCC->APB2ENR |= (1 << 12);
	delay(1e-5);

	SPI1->CR1 = (1 << 12);
	SPI1->CFG1 = (0b100 << 28) | (1 << 15) | (1 << 14) | (7 << 0);
	SPI1->CFG2 = (1 << 31) | (1 << 30) | (1 << 26) | (1 << 25) | (1 << 24) | (1 << 22);
	SPI1->IFCR = (1 << 9);
	SPI1->CR1 = (1 << 12);
	/*SPI1初始化*/

	/*DMA初始化*/
	// 打开DMA1时钟
	RCC->AHB1ENR |= (1 << 0);
	delay(1e-5);

	// DMA1_Stream0 SPI1 TX
	DMA1_Stream0->PAR = (uint32_t)&SPI1->TXDR;
	DMA1_Stream0->NDTR = 2;
	DMAMUX1_Channel0->CCR = (38 << 0);
	DMA1_Stream0->CR = (3 << 16) | (0 << 13) | (0 << 10) | (0 << 9) | (0b01 << 6);
	DMA1_Stream0->FCR = (1 << 2) | (3 << 0);

	// DMA1_Stream1 SPI1 RX
	DMA1_Stream1->PAR = (uint32_t)&SPI1->RXDR;
	DMA1_Stream1->NDTR = 2;
	DMAMUX1_Channel1->CCR = (37 << 0);
	DMA1_Stream1->CR = (3 << 16) | (0 << 13) | (1 << 10) | (0 << 9) | (0b00 << 6) | (1 << 4);
	DMA1_Stream1->FCR = (1 << 2) | (3 << 0);
	NVIC_SetPriority(DMA1_Stream1_IRQn, 3);
	/*DMA初始化*/

	os_delay(0.2);

	/*IMU初始化*/
	BMI088_Init();
	ICM20689_Init();

	if (imuIndCounter == 0)
	{ // 无imu报错
		while (1)
		{
			os_delay(0.1);
		}
	}
	/*IMU初始化*/

	// SPL06初始化
	SPL06_Init();

	/*打开TIM16 17用作传感器定时器*/
	RCC->APB2ENR |= (1 << 18) | (1 << 17);
	os_delay(1e-2);
	// 分频至10Mhz
	TIM16->PSC = TIM17->PSC = (APB2TIMERCLK / 10000000) - 1;
	// TIM16 BMI088陀螺2000hz中断
	TIM16->ARR = 10e6 / 2000 - 1;
	// TIM17 BMI088加速度800hz中断
	TIM17->ARR = 10e6 / 800 - 1;
	// 打开定时器中断
	TIM16->DIER = 1 << 0;
	TIM17->DIER = 1 << 0;
	// 打开DMA中断
	DMARxStream_Enable_IRQ;
	// 打开定时器
	TIM16->CR1 = 1 << 0;
	//		TIM17->CR1 = 1<<0;
	// 开启定时中断（定时器和DMA中断优先级必须相同）
	NVIC_SetPriority(TIM16_IRQn, 3);
	NVIC_EnableIRQ(TIM16_IRQn);
	//		NVIC_SetPriority( TIM17_IRQn , 3 );
	//		NVIC_EnableIRQ( TIM17_IRQn );
	/*打开TIM16 17用作传感器定时器*/
}
extern float debug_test[30];
// BMI088陀螺需读中断
extern "C" void TIM16_IRQHandler()
{
	TIM16->SR = 0;

	// 判断是否读SPL06
	static uint8_t SPL06_counter = 0;
	if (++SPL06_counter >= 66)
	{
		SPL06_counter = 0;
		nd_read_SPL06 = true;
	}

	static uint8_t icm20689_counter = 0;
	if (bmi088Ind >= 0)
	{ // bmi088
		static uint8_t BMI088_Acc_counter = 0;
		if (++BMI088_Acc_counter >= 2)
		{
			BMI088_Acc_counter = 0;
			nd_read_bmi088Accel = true;
		}
		if (++icm20689_counter >= 2)
		{
			icm20689_counter = 0;
			nd_read_icm20689 = true;
		}

		if (current_read_sensor == -1)
		{ // SPI总线空闲
			// 进行读操作
			PullDown_BMI088Gyro_CS;
			current_read_sensor = 0;
			TxBuf[0] = (1 << 7) | 0x02;
			SPI1_TransmitReceive_Start(TxBuf, RxBuf, sizeof(__BMI088_GyroData));
		}
		else
			// SPI总线正在读
			nd_read_bmi088Gyro = true;
	}
	else if (icm20689Ind >= 0)
	{ // ICM20689
		if (++icm20689_counter >= 2)
		{
			icm20689_counter = 0;

			if (current_read_sensor == -1)
			{ // SPI总线空闲
				// 进行读操作
				PullDown_ICM20689_CS;
				current_read_sensor = 20;
				TxBuf[0] = (1 << 7) | 59;
				SPI1_TransmitReceive_Start(TxBuf, RxBuf, sizeof(__ICM20689_Data));
			}
			else
				// SPI总线正在读
				nd_read_icm20689 = true;
		}
	}
}

static __BMI088_GyroData BMI088_GyroDataBuffer;
static void bmi088GyroTCB(void *pvParameter1, uint32_t ulParameter2)
{
	vector3<int32_t> data;
	volatile __BMI088_GyroData *datap = &BMI088_GyroDataBuffer;

	DMARxStream_Disable_IRQ;
	data.set_vector(datap->gyro[1], -datap->gyro[0], datap->gyro[2]);
	DMARxStream_Enable_IRQ;

	// 检测是否超量程
	bool data_error = false;
	if (data.x == std::numeric_limits<int16_t>::min() || data.y == std::numeric_limits<int16_t>::min() || data.z == std::numeric_limits<int16_t>::min() ||
		data.x == std::numeric_limits<int16_t>::max() || data.y == std::numeric_limits<int16_t>::max() || data.z == std::numeric_limits<int16_t>::max())
		data_error = true;
	// 更新传感器数据
	IMUGyroscopeUpdateTC(bmi088Ind, bmi088_gyro_key, data, data_error, IMU_Temperature[0], 0.01);
	MS_Notify_IMUGyroUpdate(bmi088Ind);
}

static __BMI088_AccelData BMI088_AccelDataBuffer;
static void bmi088AccelTCB(void *pvParameter1, uint32_t ulParameter2)
{
	vector3<int32_t> data;
	int16_t temperature;

	volatile __BMI088_AccelData *datap = &BMI088_AccelDataBuffer;
	DMARxStream_Disable_IRQ;
	data.set_vector(datap->acc[1], -datap->acc[0], datap->acc[2]);
	temperature = (((uint8_t *)&datap->temperature)[0] << 3) | (((uint8_t *)&datap->temperature)[1] >> 5);
	DMARxStream_Enable_IRQ;

	// 计算温度
	if (temperature > 1023)
		temperature -= 2048;
	IMU_Temperature[0] = temperature * 0.125 + 23;
	// 检测是否超量程
	bool data_error = false;
	if (data.x == std::numeric_limits<int16_t>::min() || data.y == std::numeric_limits<int16_t>::min() || data.z == std::numeric_limits<int16_t>::min() ||
		data.x == std::numeric_limits<int16_t>::max() || data.y == std::numeric_limits<int16_t>::max() || data.z == std::numeric_limits<int16_t>::max())
		data_error = true;
	// 更新传感器数据
	IMUAccelerometerUpdateTC(bmi088Ind, bmi088_accel_key, data, data_error, IMU_Temperature[0], 0.01);
	MS_Notify_IMUAceelUpdate(bmi088Ind);
}

static __ICM20689_Data ICM20689_DataBuffer;
static void icm20689TCB(void *pvParameter1, uint32_t ulParameter2)
{
	vector3<int32_t> acc_data;
	vector3<int32_t> gyro_data;
	int16_t temperature;

	volatile __ICM20689_Data datap = ICM20689_DataBuffer;
	DMARxStream_Disable_IRQ;
	gyro_data.set_vector((int16_t)__REV16(datap.gyro[1]),
						 -1 * (int16_t)__REV16(datap.gyro[0]),
						 (int16_t)__REV16(datap.gyro[2]));
	acc_data.set_vector((int16_t)__REV16(datap.acc[1]),
						-1 * (int16_t)__REV16(datap.acc[0]),
						(int16_t)__REV16(datap.acc[2]));
	temperature = __REV16(datap.temperature);
	DMARxStream_Enable_IRQ;

	IMU_Temperature[1] = temperature / 326.8f + 25;

	// 检测是否超量程
	bool data_error = false;
	if (acc_data.x == std::numeric_limits<int16_t>::min() || acc_data.y == std::numeric_limits<int16_t>::min() || acc_data.z == std::numeric_limits<int16_t>::min() ||
		acc_data.x == std::numeric_limits<int16_t>::max() || acc_data.y == std::numeric_limits<int16_t>::max() || acc_data.z == std::numeric_limits<int16_t>::max())
		data_error = true;
	// 更新传感器数据
	IMUAccelerometerUpdateTC(icm20689Ind, icm20689_accel_key, acc_data, data_error, IMU_Temperature[1], 0.01);
	MS_Notify_IMUAceelUpdate(icm20689Ind);

	// 检测是否超量程
	data_error = false;
	if (gyro_data.x == std::numeric_limits<int16_t>::min() || gyro_data.y == std::numeric_limits<int16_t>::min() || gyro_data.z == std::numeric_limits<int16_t>::min() ||
		gyro_data.x == std::numeric_limits<int16_t>::max() || gyro_data.y == std::numeric_limits<int16_t>::max() || gyro_data.z == std::numeric_limits<int16_t>::max())
		data_error = true;
	// 更新传感器数据
	IMUGyroscopeUpdateTC(icm20689Ind, icm20689_gyro_key, gyro_data, data_error, IMU_Temperature[1], 0.01);
	MS_Notify_IMUGyroUpdate(icm20689Ind);
}

static __SPL06_Data SPL06_DataBuffer;
float pressure_internal = 0;
float pressure_internal_temperature = 0;
static void SPL06TCB(void *pvParameter1, uint32_t ulParameter2)
{
	uint8_t initial_counter;
	int32_t buf32[2];
	volatile __SPL06_Data *datap = &SPL06_DataBuffer;

	DMARxStream_Disable_IRQ;
	buf32[0] = __REV(datap->pressure) >> 8;
	buf32[1] = __REV(datap->temperature) >> 8;
	DMARxStream_Enable_IRQ;

	buf32[0] = (buf32[0] & 0x800000) ? (0xFF000000 | buf32[0]) : buf32[0];
	buf32[1] = (buf32[1] & 0x800000) ? (0xFF000000 | buf32[1]) : buf32[1];

	double fPsc = buf32[0] * SPL06_coefficients.KP;
	double fTsc = buf32[1] * SPL06_coefficients.KT;
	double qua2 = SPL06_coefficients.c10 + fPsc * (SPL06_coefficients.c20 + fPsc * SPL06_coefficients.c30);
	double qua3 = fTsc * fPsc * (SPL06_coefficients.c11 + fPsc * SPL06_coefficients.c21);

	double pressure = SPL06_coefficients.c00 + fPsc * qua2 + fTsc * SPL06_coefficients.c01 + qua3;
	double temperature = SPL06_coefficients.c0 * 0.5 + SPL06_coefficients.c1 * fTsc;
	IMU_Temperature[2] = temperature;
	internal_barometer_temperature = temperature;
	internal_barometer_pressure = pressure;
	pressure_internal_temperature = IMU_Temperature[2] = temperature;
	pressure_internal = pressure;
	// 更新传感器数据
	if (pressure > 0)
	{
		double velx = get_VelocityENU_Ctrl_x();
		double vely = get_VelocityENU_Ctrl_y();
		double trust = 300; // + constrain( 0.5*safe_sqrt( velx*velx + vely*vely ), 150.0 );
		double LTtrust = 50;

		vector3<double> position;
		position.z = 4430000 * (1.0 - pow(pressure / 101325.0, 1.0 / 5.256));
		internal_barometer_altitude = position.z;
		PositionSensorUpdatePosition(internal_baro_sensor_index, internal_barometer_key, position,
									 true,	 // available
									 -1,	 // delay
									 -1,	 // xy trust
									 trust,	 // z trust
									 0,		 // addition inf
									 -1,	 // xy LTtrust
									 LTtrust // z LTtrust
		);
	}
}

extern "C" void DMA1_Stream1_IRQHandler()
{
	DMA1->LIFCR = (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8) | (1 << 6);

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// 处理数据
	switch (current_read_sensor)
	{
	case 0: // Bmi088 Gyro
	{
		PullUp_BMI088Gyro_CS;
		BMI088_GyroDataBuffer = *(__BMI088_GyroData *)RxBuf;
		xTimerPendFunctionCallFromISR(bmi088GyroTCB, 0, 0, &xHigherPriorityTaskWoken);
		break;
	}
	case 1: // Bmi088 Accel
	{
		PullUp_BMI088Accel_CS;
		BMI088_AccelDataBuffer = *(__BMI088_AccelData *)RxBuf;
		xTimerPendFunctionCallFromISR(bmi088AccelTCB, 0, 0, &xHigherPriorityTaskWoken);
		break;
	}

	case 20: // Icm20689
	{
		PullUp_ICM20689_CS;
		ICM20689_DataBuffer = *(__ICM20689_Data *)RxBuf;
		xTimerPendFunctionCallFromISR(icm20689TCB, 0, 0, &xHigherPriorityTaskWoken);
		break;
	}

	case 16: // SPL06读结束
	{
		PullUp_SPL06_CS;
		SPL06_DataBuffer = *(__SPL06_Data *)RxBuf;
		xTimerPendFunctionCallFromISR(SPL06TCB, 0, 0, &xHigherPriorityTaskWoken);
		break;
	}
	}

	// 读剩下的传感器
	if (nd_read_bmi088Gyro)
	{
		PullDown_BMI088Gyro_CS;
		nd_read_bmi088Gyro = false;
		current_read_sensor = 0;
		TxBuf[0] = (1 << 7) | 0x02;
		SPI1_TransmitReceive_Start(TxBuf, RxBuf, sizeof(__BMI088_GyroData));
	}
	else if (nd_read_bmi088Accel)
	{
		PullDown_BMI088Accel_CS;
		nd_read_bmi088Accel = false;
		current_read_sensor = 1;
		TxBuf[0] = (1 << 7) | 0x12;
		SPI1_TransmitReceive_Start(TxBuf, RxBuf, sizeof(__BMI088_AccelData));
	}
	else if (nd_read_icm20689)
	{
		PullDown_ICM20689_CS;
		nd_read_icm20689 = false;
		current_read_sensor = 20;
		TxBuf[0] = (1 << 7) | 59;
		SPI1_TransmitReceive_Start(TxBuf, RxBuf, sizeof(__ICM20689_Data));
	}
	else if (nd_read_SPL06)
	{
		PullDown_SPL06_CS;
		nd_read_SPL06 = false;
		current_read_sensor = 16;
		TxBuf[0] = (1 << 7) | 0x00;
		SPI1_TransmitReceive_Start(TxBuf, RxBuf, sizeof(__SPL06_Data));
	}
	else
	{
		current_read_sensor = -1;
	}
exit_tcirq:
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*SPI操作*/
static void SPI1_Transmit_Start(const uint8_t *TxBuf, uint16_t size)
{
	// 关闭DMA通道
	DMA1_Stream0->CR &= ~(1 << 0);
	DMA1_Stream1->CR &= ~(1 << 0);

	// 打开Tx MINC，关闭Rx MINC
	DMATxStream_Enable_MINC;
	DMARxStream_Disable_MINC;

// 清空D-Cache
#ifdef DCACHE_SIZE
	SCB_CleanDCache_by_Addr((uint32_t *)TxBuf, size);
#endif

	// 关闭SPI外设
	SPI1->CR1 &= ~(1 << 0);
	// 关闭DMA Request
	SPI1->CFG1 &= ~((1 << 15) | (1 << 14));
	// 清空DMA状态
	DMA1->LIFCR = (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 0);
	DMA1->LIFCR = (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8) | (1 << 6);
	// 设置DMA存储器地址
	DMA1_Stream0->M0AR = (uint32_t)TxBuf;
	DMA1_Stream1->M0AR = (uint32_t)RxBuf;
	// 清空SPI状态
	SPI1->IFCR = (1 << 9) | (1 << 4);
	// 设置DMA传输数量
	DMA1_Stream0->NDTR = DMA1_Stream1->NDTR = SPI1->CR2 = size;
	// 使能Rx DMA
	DMA1_Stream1->CR |= (1 << 0);
	// 打开Rx DMA Request
	SPI1->CFG1 |= (1 << 14);
	// 使能Tx DMA
	DMA1_Stream0->CR |= (1 << 0);
	// 打开Tx DMA Request
	SPI1->CFG1 |= (1 << 15);
	// 打开SPI外设开始传输
	SPI1->CR1 |= (1 << 0);
	SPI1->CR1 |= (1 << 9);
}
static void SPI1_Transmit(const uint8_t *TxBuf, uint16_t size)
{
	SPI1_Transmit_Start(TxBuf, size);
	// 等待DMA传输完成
	while ((DMA1->LISR & (1 << 11)) == 0)
		;
}

static void SPI1_TransmitReceive_Start(const uint8_t *TxBuf, uint8_t *RxBuf, uint16_t size)
{
	// 关闭DMA通道
	DMA1_Stream0->CR &= ~(1 << 0);
	DMA1_Stream1->CR &= ~(1 << 0);

	// 关闭Tx MINC，打开Rx MINC
	DMATxStream_Disable_MINC;
	DMARxStream_Enable_MINC;

// 清空D-Cache
#ifdef DCACHE_SIZE
	SCB_CleanDCache_by_Addr((uint32_t *)TxBuf, 1);
	SCB_InvalidateDCache_by_Addr((uint32_t *)RxBuf, size);
#endif

	// 关闭SPI外设
	SPI1->CR1 &= ~(1 << 0);
	// 关闭DMA Request
	SPI1->CFG1 &= ~((1 << 15) | (1 << 14));
	// 清空DMA状态
	DMA1->LIFCR = (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 0);
	DMA1->LIFCR = (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8) | (1 << 6);
	// 设置DMA存储器地址
	DMA1_Stream0->M0AR = (uint32_t)TxBuf;
	DMA1_Stream1->M0AR = (uint32_t)RxBuf;
	// 清空SPI状态
	SPI1->IFCR = (1 << 9) | (1 << 4);
	// 设置DMA传输数量
	DMA1_Stream0->NDTR = DMA1_Stream1->NDTR = SPI1->CR2 = size;
	// 使能Rx DMA
	DMA1_Stream1->CR |= (1 << 0);
	// 打开Rx DMA Request
	SPI1->CFG1 |= (1 << 14);
	// 使能Tx DMA
	DMA1_Stream0->CR |= (1 << 0);
	// 打开Tx DMA Request
	SPI1->CFG1 |= (1 << 15);

	// 打开SPI外设开始传输
	SPI1->CR1 |= (1 << 0);
	SPI1->CR1 |= (1 << 9);
}
static void SPI1_TransmitReceive(const uint8_t *TxBuf, uint8_t *RxBuf, uint16_t size)
{
	SPI1_TransmitReceive_Start(TxBuf, RxBuf, size);
	// 等待DMA传输完成
	while ((DMA1->LISR & (1 << 11)) == 0)
	{
	}
}
/*SPI操作*/
