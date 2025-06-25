/***********************************************
大连民族大学 创一工作室 队内赛模板
由轮趣科技C30D模板提供参考 标准库 FreeRTOS
系统调试程序 sys.c
功能：系统级函数
	1.系统变量初始化
	2.系统版本信息获取
	3.中断使能/禁止
	4.WIFI指令执行
	5.栈顶地址设置
	6.延时函数
	7.系统变量调试输出函数
	8.系统变量调试输出函数（可选）
	9.系统变量调试输出函数（可选）
版本：V1.0
作者：TuxMonkey
日期：2025.5.17
Copyright (c) 2025 DLMU-CONE
***********************************************/
#include "sys.h"  

//用于调试的系统变量
DEBUG_t debug;

//输入版本参数,返回版本信息函数
const uint8_t* getSW_Ver(SOFTWARE_VERSION ver)
{
	switch( ver )
	{
		case V1_00: return "V1.00";
		//待定后续版本.
	}
	
	return "unknown";
}

const uint8_t* getHW_Ver(HARDWARE_VERSION ver)
{
	switch( ver )
	{
		case V1_0: return "V1.0";
		case V1_1: return "V1.1";
		//待定后续版本.
	}
	
	return "unknown";
}


//系统级变量初始化
//设置软硬件版本信息.硬件版本由硬件初始化后才能确定.
void SYS_VAL_t_Init(SYS_VAL_t* p)
{
	//当前软件版本
	p->SoftWare_Ver = V1_00;
	
	p->HardWare_Ver = HW_NONE;//待硬件初始化后确定
	
	p->Time_count = 0;
	p->HardWare_charger = 0;
	p->SecurityLevel = 1; //系统安全等级,默认设置1.不带命令丢失保护
	p->LED_delay = 1; //系统状态指示灯闪烁间隔,单位 ms
}




//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//关闭所有中断(但是不包括fault和NMI中断)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//开启所有中断
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}
















