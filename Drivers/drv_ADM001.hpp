#pragma once

#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

extern bool ADM001_last_read_ok;
extern uint16_t ADM001_recv_sum;
extern uint16_t ADM001_calc_sum;
extern float weight_kg;

void init_drv_ADM001();
