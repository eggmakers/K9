#include "Basic.hpp"
#include "drv_IIC_IRLock.hpp"
#include "drv_ExtIIC.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "MeasurementSystem.hpp"
#include "precLand.hpp"
#include "followTarget.hpp"

#define IRLOCK_I2C_ADDRESS      0x54
#define IRLOCK_SYNC         0xAA55AA55

typedef struct
{
	uint16_t checksum;
	uint16_t signature;
	uint16_t pixel_x;
	uint16_t pixel_y;
	uint16_t pixel_size_x;
	uint16_t pixel_size_y;
}__PACKED _IRFrame;

static void pixel_to_1M_plane(float pix_x, float pix_y, float &ret_x, float &ret_y)
{
    ret_x = (-0.00293875727162397f*pix_x + 0.470201163459835f)/(4.43013552642296e-6f*((pix_x - 160.0f)*(pix_x - 160.0f)) +
                                                                4.79331390531725e-6f*((pix_y - 100.0f)*(pix_y - 100.0f)) - 1.0f);
    ret_y = (-0.003056843086277f*pix_y + 0.3056843086277f)/(4.43013552642296e-6f*((pix_x - 160.0f)*(pix_x - 160.0f)) +
                                                            4.79331390531725e-6f*((pix_y - 100.0f)*(pix_y - 100.0f)) - 1.0f);
}


static bool sync_frame_start(uint8_t* rx_buf)
{
	if( ExtIIC_ReceiveAddr7( IRLOCK_I2C_ADDRESS, rx_buf, 4 ) == false )
		return false;
	uint32_t sync_word = *(uint32_t*)rx_buf;

	uint8_t count=40;
	while (count-- && sync_word != IRLOCK_SYNC && sync_word != 0)
	{
		if( ExtIIC_ReceiveAddr7( IRLOCK_I2C_ADDRESS, rx_buf, 1 ) == false )
			return false;
		
		if (rx_buf[0] == 0)
			break;
		
		sync_word = (sync_word>>8) | (uint32_t(rx_buf[0])<<24);
	}
	return sync_word == IRLOCK_SYNC;
}

static bool read_block(uint8_t* rx_buf)
{
	if( ExtIIC_ReceiveAddr7( IRLOCK_I2C_ADDRESS, rx_buf, sizeof(_IRFrame) ) == false )
			return false;

	_IRFrame* frame = (_IRFrame*)rx_buf;
	
	/* check crc */
	uint32_t crc = frame->signature + frame->pixel_x + frame->pixel_y + frame->pixel_size_x + frame->pixel_size_y;
	if (crc != frame->checksum) {
		// printf("bad crc 0x%04x 0x%04x\n", crc, irframe.checksum);
		return false;
	}
	return true;
}

static void IRLock_Server(void* pvParameters)
{
	Aligned_DMABuf uint8_t tx_buf[32];
	Aligned_DMABuf uint8_t rx_buf[32];
	
	//注册精准降落传感器
	uint32_t followKey = followSensorRegister(followDataType_track_s_xy);
	
reTry:
	while(1)
	{	
		bool res = sync_frame_start(rx_buf);
		if(res)
			res = read_block(rx_buf);
		
		if( res )
		{
			_IRFrame* frame = (_IRFrame*)rx_buf;
			int16_t corner1_pix_x = frame->pixel_x - frame->pixel_size_x/2;
			int16_t corner1_pix_y = frame->pixel_y - frame->pixel_size_y/2;
			int16_t corner2_pix_x = frame->pixel_x + frame->pixel_size_x/2;
			int16_t corner2_pix_y = frame->pixel_y + frame->pixel_size_y/2;

			float corner1_pos_x, corner1_pos_y, corner2_pos_x, corner2_pos_y;
			pixel_to_1M_plane(corner1_pix_x, corner1_pix_y, corner1_pos_x, corner1_pos_y);
			pixel_to_1M_plane(corner2_pix_x, corner2_pix_y, corner2_pos_x, corner2_pos_y);

			float ang_x = -0.5f*(corner1_pos_x+corner2_pos_x);
			float ang_y = -0.5f*(corner1_pos_y+corner2_pos_y);
			
			Quaternion quat;
			get_Airframe_quat(&quat);
			float rol = quat.getRoll();
			float pit = quat.getPitch();
			
//			extern float debug_test[30];
//			debug_test[25] = ang_y - pit;
//			debug_test[26] = ang_x + rol;
			
			PosSensorHealthInf1 ZRange_inf;
			if( get_OptimalRange_Z(&ZRange_inf) && ZRange_inf.last_healthy_TIME.is_valid() && ZRange_inf.last_healthy_TIME.get_pass_time()<50 )
			{	//测距传感器可用
				
				vector3<double> Position;
				get_Position_Ctrl(&Position);
				
				//获取高度
				double height = ZRange_inf.HOffset + ZRange_inf.PositionENU.z;
				
				
				vector3<double> pos;
				pos.x = Position.x + (ang_y - pit) * height;
				pos.y = Position.y + (ang_x + rol) * height;
				pos.z = -height;
				vector3<double> vel;
				double yaw = 0;//quat.getYaw() - degree2rad((double)SensorD.d);
				update_followSensor( followKey, pos, vel, yaw, true );
				
				extern float debug_test[30];
				debug_test[25] = pos.x;
				debug_test[26] = pos.y;
			}
			else
				setInavailable_followSensor(followKey);
		}
		else
			setInavailable_followSensor(followKey);
		
//		/* convert to angles */
//		_target_info.timestamp = AP_HAL::millis();
//		_target_info.pos_x = 0.5f*(corner1_pos_x+corner2_pos_x);
//		_target_info.pos_y = 0.5f*(corner1_pos_y+corner2_pos_y);
//		_target_info.pos_z = 1.0f;

		os_delay(0.1);
	}
}

static bool I2C_IRLock_DriverInit()
{
	return true;
}
static bool I2C_IRLock_DriverRun()
{
	xTaskCreate( IRLock_Server, "IICIRLock", 800, NULL, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_IICIRLock()
{
	I2CFunc_Register( 97, I2C_IRLock_DriverInit, I2C_IRLock_DriverRun );
}
