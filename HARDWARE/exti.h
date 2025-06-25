/**
 * @file exti.h
 * @author TuxMonkey (nqx_2004@qq.com)
 * @brief External interrupt control
 * @version 0.1
 * @date 2025-05-17
 * @bug 貌似没有用到
 * Copyright (c) 2025 DLMU-CONE
 * 
 */
#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"

#define INT PBin(15)   //PB5连接到MPU6050的中断引脚
void EXTI_Init(void);	//外部中断初始化		 					    
#endif

























