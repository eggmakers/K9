#pragma once

#include "Modes.hpp"

class M16_GyroAlignCalib:public Mode_Base 
{
	private:
		
	public:
		M16_GyroAlignCalib();
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};