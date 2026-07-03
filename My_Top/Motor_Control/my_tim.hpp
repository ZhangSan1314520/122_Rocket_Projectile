#pragma once  // 保证头文件只被编译一次，防止头文件被重复引用

#include "tim.h"
#include "HAL_System.hpp"
#include "DcMotor_ClosedLoop.hpp"

void My_Tim_Init(TIM_HandleTypeDef* htim,ADC_HandleTypeDef *hadc);



