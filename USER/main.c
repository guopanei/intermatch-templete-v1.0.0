/***********************************************
大连民族大学 创一工作室 队内赛模板
由轮趣科技C30D模板提供参考 标准库 FreeRTOS
主程序 main.c
说明：主函数，创建任务
版本：V1.0
作者：TuxMonkey
日期：2025.5.17
Copyright (c) 2025 DLMU-CONE
***********************************************/

/**
 * @file main.c
 * @author TuxMonkey (nqx_2004@qq.com)
 * @brief 主程序，创建任务
 * @version 0.1
 * @date 2025-05-17
 * 
 * Copyright (c) 2025 DLMU-CONE
 * 
 */
#include "system.h"

//Task priority    //任务优先级
#define START_TASK_PRIO	1

//Task stack size //任务堆栈大小	
#define START_STK_SIZE 	256  

//Task handle     //任务句柄
TaskHandle_t StartTask_Handler;

//Task function   //任务函数
void start_task(void *pvParameters);

//Main function //主函数
int main(void)
{ 
  systemInit(); //Hardware initialization //硬件初始化
	
	//Create the start task //创建开始任务
	xTaskCreate((TaskFunction_t )start_task,            //Task function   //任务函数
							(const char*    )"start_task",          //Task name       //任务名称
							(uint16_t       )START_STK_SIZE,        //Task stack size //任务堆栈大小
							(void*          )NULL,                  //Arguments passed to the task function //传递给任务函数的参数
							(UBaseType_t    )START_TASK_PRIO,       //Task priority   //任务优先级
							(TaskHandle_t*  )&StartTask_Handler);   //Task handle     //任务句柄    					
	vTaskStartScheduler();  //Enables task scheduling //开启任务调度	
}
 
//Start task task function //开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area //进入临界区
	
    //Create the task //创建任务
	xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  NULL);	//Vehicle motion control task //小车运动控制任务
	if(SysVal.HardWare_Ver==V1_0) 	//IMU data read task //IMU数据读取任务,根据不同的硬件版本启动不同的任务.
		xTaskCreate(MPU6050_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
	else if( SysVal.HardWare_Ver==V1_1 )
		xTaskCreate(ICM20948_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
	xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     NULL); //The OLED display displays tasks //OLED显示屏显示任务
	xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      NULL);	//LED light flashing task //LED灯闪烁任务
	xTaskCreate(pstwo_task,    "PSTWO_task",    PS2_STK_SIZE,      NULL, PS2_TASK_PRIO,      NULL);	//Read the PS2 controller task //读取PS2手柄任务
	xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, DATA_TASK_PRIO,     NULL);	//Usartx3, Usartx1 and CAN send data task //串口3、串口1、CAN发送数据任务
	
    vTaskDelete(StartTask_Handler); //Delete the start task //删除开始任务

    taskEXIT_CRITICAL();            //Exit the critical section//退出临界区
}






