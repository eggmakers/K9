#include "Modes.hpp"
#include "M35_Auto1.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "NavCmdProcess.hpp"

M35_Auto1::M35_Auto1():Mode_Base( "Auto1", 35 )
{
	
}

ModeResult M35_Auto1::main_func( void* param1, uint32_t param2 )
{
	double freq = 50;
	setLedMode(LEDMode_Flying1);
	Altitude_Control_Enable();
	Position_Control_Enable();
	uint16_t exit_mode_counter_rs = 0;
	uint16_t exit_mode_counter = 0;
	uint16_t exit_mode_Gcounter = 0;
	uint32_t RollOverProtectCounter = 0;
	
	//读取模式配置
	ModeFuncCfg MFunc_cfg;
	ReadParamGroup( "MFunc", (uint64_t*)&MFunc_cfg, 0 );
	
	//任务模式
	bool RTLMode = false;
	#define change_MissionMode(x) {MissionMode=x; mode_switched = true;}
	bool MissionMode = true;
	bool mode_switched = true;
	uint8_t ModeButtonZone = 255;
	uint8_t MissionButtonZone = 255;
	uint8_t RTLButtonZone = 255;
	bool last_SfRTL = false;
	bool RTLMode_lock = false;
	bool MissionMode_lock = false;
	
	//当前执行任务的序号
	uint16_t mission_ind = 0;
	//任务状态机
	NavCmdInf navInf;
	init_NavCmdInf(&navInf);
	while(1)
	{
		os_delay(0.02);
		
		if( get_CrashedState() )
		{	//侧翻加锁
			Attitude_Control_Disable();
			return MR_Err;
		}
		
		//获取接收机
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		//获取消息
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		bool msg_handled = false;
		
		if( rc.available )
		{	//接收机可用更新模式按钮状态
			uint8_t new_ModeButtonZone;
			if( rc.available_channels >= 5 )
				new_ModeButtonZone = get_RcButtonZone( rc.data[4], ModeButtonZone );
			else
				new_ModeButtonZone = 255;
			if( new_ModeButtonZone<=5 && ModeButtonZone<=5 && new_ModeButtonZone!=ModeButtonZone )
			{	//模式按钮改变重置锁定状态
				MissionMode_lock = RTLMode_lock = false;
				last_SfRTL = false;						
			}
			ModeButtonZone = new_ModeButtonZone;
		}
		else
		{	//接收机不可用重置遥控状态
			ModeButtonZone = MissionButtonZone = RTLButtonZone = 255;
		}
		
		bool req_MissionMode = MissionMode;
		bool req_RTLMode = RTLMode;
		if( get_Position_MSStatus()==MS_Ready )
		{
			if( rc.available )
			{	//有定位且接收机可用
				//使用遥控器更新飞行模式
				bool sticks_in_neutral = 
					in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
				if( !sticks_in_neutral )
				{	//摇杆没回中不允许自动操作
					req_MissionMode = req_RTLMode = false;
					MissionMode_lock = RTLMode_lock = false;
					MissionButtonZone = RTLButtonZone = 255;
					last_SfRTL = false;
				}
				else
				{	//摇杆回中可执行自动操作	
					
					/*判断执行任务*/
						if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
						{	//按钮按下执行任务
							if( rc.available_channels >= MFunc_cfg.MissionBt[0]+4 )
							{			
								//获取按钮状态
								double btn_value = rc.data[MFunc_cfg.MissionBt[0]-1+4];
								uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );		
								if( MissionMode_lock == false )
								{	//没锁定模式时才可进行模式切换
									if( new_MissionButtonZone>=4 )
										req_MissionMode = true;	
									else
										req_MissionMode = false;
								}
								if( MissionButtonZone<=5 && new_MissionButtonZone!=MissionButtonZone )
								{	//按钮状态发生变化
									MissionMode_lock = false;								
								}
								MissionButtonZone = new_MissionButtonZone;
							}
						}
						else if( MFunc_cfg.MissionBt[0]>=12 && MFunc_cfg.MissionBt[0]<=14 )
						{	//按钮变化执行任务
							if( rc.available_channels >= MFunc_cfg.MissionBt[0]-10+4 )
							{
								//获取按钮状态
								double btn_value = rc.data[MFunc_cfg.MissionBt[0]-11+4];
								uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );
								if( MissionButtonZone<=5 && new_MissionButtonZone!=MissionButtonZone )
								{	//按钮状态发生变化
									MissionMode_lock = false;									
									req_MissionMode = !req_MissionMode;
								}
								MissionButtonZone = new_MissionButtonZone;
							}
						}
						
						if( msg_available && msg.cmd==176 )
						{	//do set mode
							if( (int)msg.params[0] & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED )
							{	//mavlink定义模式
								uint32_t main_mode = msg.params[1];
								uint32_t sub_mode = msg.params[2];
								if( (main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || main_mode==0) && sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_MISSION )
								{	//指令进入任务模式
									req_MissionMode = true;
									//锁定模式防止按钮状态不在当前模式造成马上退出
									MissionMode_lock = true;
									msg_handled = 1;
								}
							}
						}
					
						//模式按钮切换任务模式
						if( MissionMode_lock == false )
						{	//没锁定模式时才可进行模式切换
							if( ModeButtonZone==0 && MFunc_cfg.Bt1AFunc1[0]==22 )
								req_MissionMode = true;
							else if( ModeButtonZone==1 && MFunc_cfg.Bt1AFunc2[0]==22 )
								req_MissionMode = true;
							else if( ModeButtonZone==2 && MFunc_cfg.Bt1AFunc3[0]==22 )
								req_MissionMode = true;
							else if( ModeButtonZone==3 && MFunc_cfg.Bt1AFunc4[0]==22 )
								req_MissionMode = true;
							else if( ModeButtonZone==4 && MFunc_cfg.Bt1AFunc5[0]==22 )
								req_MissionMode = true;
							else if( ModeButtonZone==5 && MFunc_cfg.Bt1AFunc6[0]==22 )
								req_MissionMode = true;
						}
					/*判断执行任务*/
					
					/*判断返航*/
						//失控恢复后继续返航（打杆后退出）
						if( last_SfRTL )
							req_RTLMode = true;
						else if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
						{	//按钮按下返航
							if( rc.available_channels >= MFunc_cfg.RTLBt[0]+4 )
							{
								//获取按钮状态
								double btn_value = rc.data[MFunc_cfg.RTLBt[0]-1+4];
								uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );		
								if( RTLMode_lock == false )
								{	//没锁定模式时才可进行模式切换
									if( new_RTLButtonZone>=4 )
										req_RTLMode = true;	
									else
										req_RTLMode = false;
								}
								if( RTLButtonZone<=5 && new_RTLButtonZone!=RTLButtonZone )
								{	//按钮状态发生变化
									RTLMode_lock = false;								
								}
								RTLButtonZone = new_RTLButtonZone;
							}
						}
						else if( MFunc_cfg.RTLBt[0]>=12 && MFunc_cfg.RTLBt[0]<=14 )
						{	//按钮变化返航
							if( rc.available_channels >= MFunc_cfg.RTLBt[0]-10+4 )
							{
								//获取按钮状态
								double btn_value = rc.data[MFunc_cfg.RTLBt[0]-11+4];
								uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );	
								if( RTLButtonZone<=5 && new_RTLButtonZone!=RTLButtonZone )
								{	//按钮状态发生变化
									RTLMode_lock = false;									
									req_RTLMode = !RTLMode;
								}
								RTLButtonZone = new_RTLButtonZone;
							}
						}
						
						if( msg_available && msg.cmd==176 )
						{	//do set mode
							if( (int)msg.params[0] & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED )
							{	//mavlink定义模式
								uint32_t main_mode = msg.params[1];
								uint32_t sub_mode = msg.params[2];
								if( (main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || main_mode==0) && sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_RTL )
								{	//指令进入任务模式
									req_RTLMode = true;
									//锁定模式防止按钮状态不在当前模式造成马上退出
									RTLMode_lock = true;
									msg_handled = 1;
								}
							}
						}
						
						//模式按钮切换返航模式
						if( RTLMode_lock == false )
						{	//没锁定模式时才可进行模式切换
							if( ModeButtonZone==0 && MFunc_cfg.Bt1AFunc1[0]==23 )
								req_RTLMode = true;
							else if( ModeButtonZone==1 && MFunc_cfg.Bt1AFunc2[0]==23 )
								req_RTLMode = true;
							else if( ModeButtonZone==2 && MFunc_cfg.Bt1AFunc3[0]==23 )
								req_RTLMode = true;
							else if( ModeButtonZone==3 && MFunc_cfg.Bt1AFunc4[0]==23 )
								req_RTLMode = true;
							else if( ModeButtonZone==4 && MFunc_cfg.Bt1AFunc5[0]==23 )
								req_RTLMode = true;
							else if( ModeButtonZone==5 && MFunc_cfg.Bt1AFunc6[0]==23 )
								req_RTLMode = true;
						}
					/*判断返航*/
				}
			}
		}
		else
		{	//无定位重置模式状态
			MissionMode_lock = RTLMode_lock = false;
			MissionButtonZone = RTLButtonZone = 255;
		}
		
		if( req_RTLMode )
		{	//进入返航模式
			RTLMode = true;
			mode_switched = true;
		}
		else
		{
			RTLMode = false;
			if( req_MissionMode )
			{	//进入任务模式
				if( MissionMode==0 )
					change_MissionMode(1);
			}
			else
			{	//退出任务模式
				if( MissionMode!=0 )
					change_MissionMode(0);
			}
		}
		
		if( RTLMode )
		{	//进入安全模式返航
			enter_MSafe(true);
			if( rc.available==false )
				last_SfRTL = true;
			/*判断退出模式*/
				bool inFlight;
				get_is_inFlight(&inFlight);
				if( inFlight==false )
				{
					Attitude_Control_Disable();
					return MR_OK;
				}
			/*判断退出模式*/
		}
		else if( MissionMode )
		{	//任务模式
			Position_Control_Enable();
			bool pos_ena;
			is_Position_Control_Enabled(&pos_ena);
			if( pos_ena == false )
			{	//位置控制器无法打开返回手动模式
				MissionMode = false;
				goto Manual_Mode;
			}
			
			if( mode_switched )
			{	//刚切换到任务模式
				//首先刹车等待
				
				if( rc.available )
				{
					bool sticks_in_neutral = 
						in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[3] , 50 , 5 );
					if( !sticks_in_neutral )
					{	//摇杆不在中间返回手动模式
						init_NavCmdInf(&navInf);
						MissionMode = false;
						goto Manual_Mode;
					}
				}
				
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				
				//等待刹车完成
				Position_ControlMode alt_mode, pos_mode;
				get_Altitude_ControlMode(&alt_mode);
				get_Position_ControlMode(&pos_mode);
				if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
				{	//刹车完成
					++navInf.counter2;
					//等待1秒再进入任务飞行
					if( navInf.counter2 >= 1*freq )
					{	//进入任务飞行模式
						mode_switched = false;
					}
				}
				else
					navInf.counter2 = 0;
			}
			else
			{	//任务飞行				
				if( rc.available )
				{
					bool sticks_in_neutral = 
						in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[3] , 50 , 5 );
					if( !sticks_in_neutral )
					{	//打杆返回手动模式
						init_NavCmdInf(&navInf);
						MissionMode = false;
						goto Manual_Mode;
					}
				}
				
				//上报模式状态
				setCurrentFlyMode(AFunc_Mission);
				
				//根据mission_ind状态判断当前需要执行什么飞行动作
				switch( mission_ind )
				{
					case 0:
					{	//起飞
						double params[7];
						params[0] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0;
						params[6] = 0.5;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_TAKEOFF, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//起飞完成
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 1:
					{	//巡线
//						double vec[3];
//						vec[0] = msg.cmd;
//						vec[1] = msg.params[0];
//						vec[2] = msg.params[1];
//						SDLog_Msg_DebugVect("xun",vec,3);
						
						Position_Control_set_ZLock();
						if(msg_available)
						{		
							if( msg.cmd == 2 )
							{
								if(msg.params[0]!=200)
								{
//									//roll角
//									Quaternion quat;
//									get_history_AirframeQuatY(&quat,0.1);
//									double roll = quat.getRoll();
//									//对地高度
//									double height = 100;
//									PosSensorHealthInf1 ZRange_inf;
//									if( get_OptimalRange_Z( &ZRange_inf ) )
//									{	//测距传感器可用
//										if( ZRange_inf.last_healthy_TIME.is_valid() && ZRange_inf.last_healthy_TIME.get_pass_time() < 50 )
//										{	//测距50秒内健康
//											//获取高度
//											height = ZRange_inf.HOffset + ZRange_inf.PositionENU.z;
//										}
//									}
//									//倾角补偿
//									double
									
									//逆时针为正角度误差（度）
									double angle_err = -msg.params[0];
									double angle_err_rad = degree2rad(angle_err);
									//距离误差向左为正（无单位）
									double pos_err = -msg.params[1];
									//修正偏航头朝直线
									Attitude_Control_set_Target_YawRate( degree2rad(constrain( angle_err, 70.0 ) ));
									//求巡线压线修正速度
									vector2<double> d_vel;
									d_vel.y = pos_err*cos(angle_err_rad);
									d_vel.x = pos_err*sin(-angle_err_rad);
									d_vel *= 2.5;
									d_vel.constrain(150);
									//求巡线前向速度
									vector2<double> f_vel;
									f_vel.y = sin(angle_err_rad);
									f_vel.x = cos(angle_err_rad);
									f_vel *= constrain( ( 30 - fabs(angle_err) ), 3.0, 12.0 );
									Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( d_vel.x+f_vel.x, d_vel.y+f_vel.y , 0.05, 0.05);
								}
								if(msg.params[0] == 200)
								{
									Attitude_Control_set_YawLock();
									Position_Control_set_XYLock();
								}
							}
						}
						break;
					}
					
//					case 1:
//					{	//飞直线
//						double params[7];
//						params[0] = 2;
//						params[1] = 0;
//						params[2] = 0;
//						params[3] = nan("");
//						params[4] = 0.5;	params[5] = 0;
//						params[6] = 0.5;
//						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
//						if( NavCmdRs_SuccessOrFault(res) )
//						{	//飞直线完成
//							init_NavCmdInf(&navInf);
//							++mission_ind;
//						}
//						break;
//					}
					
					case 2:
					{	//飞直线
						double params[7];
						params[0] = 2;
						params[1] = 0;
						params[2] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0.5;
						params[6] = 0;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//飞直线完成
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 3:
					{	//飞直线
						double params[7];
						params[0] = 2;
						params[1] = 0;
						params[2] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0.5;
						params[6] = 0;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//飞直线完成
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 4:
					{	//飞直线
						double params[7];
						params[0] = 2;
						params[1] = 0;
						params[2] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0.5;
						params[6] = -0.5;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//飞直线完成
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 5:
					{	//降落
						Position_Control_set_XYLock();
						Position_Control_set_TargetVelocityZ(-50);
						break;
					}
					
					default:
					{
						MissionMode = false;
						mission_ind = 0;
						goto Manual_Mode;
						break;
					}
				}
			}
		}
		else
		{	//手动飞行模式（包含定高定点控制）
			Manual_Mode:
			if( rc.available )
			{				
				/*判断退出模式*/
					//获取飞行状态
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( rc.data[0] > 30 )
					{
						exit_mode_counter_rs = 480;
						if( exit_mode_counter < exit_mode_counter_rs )
							exit_mode_counter = exit_mode_counter_rs;
					}
					//降落后自动加锁
					if( inFlight==false && rc.data[0]<30 )
					{
						if( ++exit_mode_counter >= 500 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_counter = exit_mode_counter_rs;
					//手势强制加锁
					if( rc.data[0] < 10 && rc.data[1] < 10 && rc.data[2] < 10 && rc.data[3] > 90 )
					{
						if( ++exit_mode_Gcounter >= 50 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_Gcounter = 0;
				/*判断退出模式*/
				
				/*判断模式*/
					uint8_t MF_mode = 0;
					if( rc.data[4]<1.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc1[0];
					else if( rc.data[4]<2.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc2[0];
					else if( rc.data[4]<3.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc3[0];
					else if( rc.data[4]<4.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc4[0];
					else if( rc.data[4]<5.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc5[0];
					else if( rc.data[4]<6.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc6[0];
				/*判断模式*/
					
				//切换定高定点
				if( MF_mode==2 )
					Position_Control_Enable();
				else if( MF_mode==1 )
					Position_Control_Disable();
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
					
				//油门杆控制垂直速度
				if( in_symmetry_range_mid( rc.data[0] , 50 , 5 ) )
					Position_Control_set_ZLock();
				else
				{
					double thr_stick = remove_deadband( rc.data[0] - 50.0 , 5.0 );
					if( thr_stick > 0 )
						thr_stick *= getPosCtrlCfg()->maxVelUp[0] / 50;
					else
						thr_stick *= getPosCtrlCfg()->maxVelDown[0] / 50;
					Position_Control_set_TargetVelocityZ(thr_stick);
				}
				
				if( pos_ena )
				{
					//上报模式状态
					setCurrentFlyMode(AFunc_PosHold);
					
					//俯仰横滚杆控水平速度
					if( in_symmetry_range_mid( rc.data[3] , 50 , 5 ) && in_symmetry_range_mid( rc.data[2] , 50 , 5 ) )
						Position_Control_set_XYLock();
					else
					{
						double RPCtrlScale = atan2( getPosCtrlCfg()->maxAccXY[0] / 50.0, GravityAcc );
						double XYCtrlScale = getPosCtrlCfg()->maxVelXY[0] / 50.0;						
						double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, 5.0 );
						double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, 5.0 );
						vector3<double> velocityFLU;
						get_VelocityFLU_Ctrl(&velocityFLU);
						double vel_stick_err_pitch = velocityFLU.x/XYCtrlScale - pitch_sitck_d;
						double vel_stick_err_roll = velocityFLU.y/XYCtrlScale - -roll_sitck_d;
						Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
							pitch_sitck_d * XYCtrlScale ,\
							-roll_sitck_d * XYCtrlScale , \
							fabs( vel_stick_err_roll  )*RPCtrlScale, \
							fabs( vel_stick_err_pitch )*RPCtrlScale \
						);
					}
				}
				else
				{
					//上报模式状态
					setCurrentFlyMode(AFunc_AltHold);
					
					//补偿风力扰动
					vector3<double> WindDisturbance;
					get_WindDisturbance( &WindDisturbance );
					Quaternion attitude;
					get_Attitude_quat(&attitude);
					double yaw = attitude.getYaw();		
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
					double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					//俯仰横滚杆控俯仰横滚
					double RPCtrlScale = degree2rad( getAttCtrlCfg()->maxLean[0] / 50.0 );
	//				Attitude_Control_set_Target_RollPitch( 
	//					( rc.data[3] - 50 )*RPCtrlScale - atan2(-WindDisturbance_Bodyheading_y , GravityAcc ),
	//					( rc.data[2] - 50 )*RPCtrlScale - atan2( WindDisturbance_Bodyheading_x , GravityAcc ) 
	//				);
					Attitude_Control_set_Target_RollPitch( 
						( rc.data[3] - 50 )*RPCtrlScale,
						( rc.data[2] - 50 )*RPCtrlScale
					);
				}
				
				//偏航杆在中间锁偏航
				//不在中间控制偏航速度
				double YCtrlScale = degree2rad( getAttCtrlCfg()->maxYSp[0] / 50.0 );
				if( in_symmetry_range_mid( rc.data[1] , 50 , 5 ) )
					Attitude_Control_set_YawLock();
				else
					Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );
			}
			else
			{
				last_SfRTL = true;
				//无遥控信号进入安全模式
				enter_MSafe();
				/*判断退出模式*/
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( inFlight==false )
					{
						Attitude_Control_Disable();
						return MR_OK;
					}
				/*判断退出模式*/
				
			}
		}
	}
	return MR_OK;
}