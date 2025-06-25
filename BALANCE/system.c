/**
 * @file system.c
 * @author TuxMonkey (nqx_2004@qq.com)
 * @brief 系统初始化和变量设置
 * @version 0.1
 * @date 2025-05-17
 * 
 * Copyright (c) 2025 DLMU-CONE
 * 
 */

#include "system.h"

//Robot software fails to flag bits
//机器人软件失能标志位
u8 Flag_Stop=0;

//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models
//ADC值分段变量，取决于小车型号数量，目前有6种小车型号
int Divisor_Mode;

// Robot type variable
//机器人型号变量
//0=Mec_Car，1=Omni_Car，2=Akm_Car，3=Diff_Car，4=FourWheel_Car，5=Tank_Car
u8 Car_Mode=0;

//Servo control PWM value, Ackerman car special
//舵机控制PWM值，阿克曼小车专用
int Servo;

//Default speed of remote control car, unit: mm/s
//遥控小车的默认速度，单位：mm/s
float RC_Velocity=500;

//Vehicle three-axis target moving speed, unit: m/s
//小车三轴目标运动速度，单位：m/s
float Move_X, Move_Y, Move_Z;

//PID parameters of Speed control
//速度控制PID参数
float Velocity_KP=300,Velocity_KI=900;

//Smooth control of intermediate variables, dedicated to omni-directional moving cars
//平滑控制中间变量，全向移动小车专用
Smooth_Control smooth_control;

//The parameter structure of the motor
//电机的参数结构体
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;

/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//编码器精度
float Encoder_precision;
//Wheel circumference, unit: m
//轮子周长，单位：m
float Wheel_perimeter;
//Drive wheel base, unit: m
//主动轮轮距，单位：m
float Wheel_spacing;
//The wheelbase of the front and rear axles of the trolley, unit: m
//小车前后轴的轴距，单位：m
float Axle_spacing;
//All-directional wheel turning radius, unit: m
//全向轮转弯半径，单位：m
float Omni_turn_radiaus;
/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1, serial port 5 communication control flag bit.
//These 6 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2手柄、蓝牙APP、航模手柄、CAN通信、串口1、串口5通信控制标志位。这6个标志位默认都为0，代表串口3控制模式
u8 PS2_ON_Flag=0, APP_ON_Flag=0, Remote_ON_Flag=0, CAN_ON_Flag=0, Usart1_ON_Flag, Usart5_ON_Flag;

//Bluetooth remote control associated flag bits
//蓝牙遥控相关的标志位
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag;

//Sends the parameter's flag bit to the Bluetooth APP
//向蓝牙APP发送参数的标志位
u8 PID_Send;

//The PS2 gamepad controls related variables
//PS2手柄控制相关变量
float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;

//Self-check the relevant flag variables
//自检相关标志变量
int Check=0, Checking=0, Checked=0, CheckCount=0, CheckPhrase1=0, CheckPhrase2=0;

//Check the result code
//自检结果代码
long int ErrorCode=0;

u8 Get_Charging_HardWare=0;//是否存在自动回充装备
u8 charger_check=0;//实时检测充电装备在线状态

USBH_HOST  USB_Host;
USB_OTG_CORE_HANDLE  USB_OTG_Core_dev;

//系统相关变量
SYS_VAL_t SysVal;

//Control or self-check code flag bit
//控制or自检代码标志位
u8 Proc_Flag=0; 

//舵机计数值
int servo_direction[6] = {0};
int servo_pwm_count = 500;

int check_time_count_motor_forward=300,check_time_count_motor_retreat=500;
int POT_val;

int Servo_Count[6] = {500, 500, 500, 500, 500, 500};

u8 uart3_receive_message[50];
u8 uart3_send_flag;
u8 message_count=0;

u8 uart2_receive_message[50];
u8 uart2_send_flag=5;
u8 app_count=0;

int Full_rotation = 16799;

void systemInit(void)
{

	//Interrupt priority group setti  ng
	//中断优先级分组设置
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	//Delay function initialization
	//延时函数初始化
    delay_init(168);
	//IIC initialization for IMU
    //IIC初始化，用于IMU
    I2C_GPIOInit();
	//系统相关软件参数初始化
	SYS_VAL_t_Init(&SysVal);
	
	//如果IMU为MPU6050,则是旧版C30D
	if( MPU6050_DEFAULT_ADDRESS == MPU6050_getDeviceID() )
	{
		SysVal.HardWare_Ver = V1_0;
		//Initialize the hardware interface to the PS2 controller
		//初始化与PS2手柄连接的硬件接口
		PS2_Init();
		//PS2 gamepad configuration is initialized and configured in analog mode
		//PS2手柄配置初始化,配置为模拟量模式	
		PS2_SetInit();		 
		//Initialize the hardware interface connected to the LED lamp
		//初始化与LED灯连接的硬件接口
		V1_0_LED_Init(); 
		//MPU6050 is initialized to read the vehicle's three-axis attitude,
		//three-axis angular velocity and three-axis acceleration information
		//MPU6050初始化，用于读取小车三轴角速度、三轴加速度信息
		MPU6050_initialize();
	}
	//如果IMU型号为ICM20948,则是新版C30D
	else if( REG_VAL_WIA == ICM20948_getDeviceID() )//读取ICM20948 id
	{
		SysVal.HardWare_Ver = V1_1;
		//USB PS2初始化
		USBH_Init(&USB_OTG_Core_dev,USB_OTG_FS_CORE_ID,&USB_Host,&HID_cb,&USR_Callbacks);
		//Initialize the hardware interface connected to the LED lamp
		//初始化与LED灯连接的硬件接口
		V1_1_LED_Init();
		//MPU6050 is initialized to read the vehicle's three-axis attitude,
		//three-axis angular velocity and three-axis acceleration information
		//ICM20948初始化，用于读取小车三轴角速度、三轴加速度信息
		invMSInit();
	}
	else //无法识别的陀螺仪,复位系统
	{
		NVIC_SystemReset();
	}         

    //Initialize the hardware interface connected to the buzzer
    //初始化与蜂鸣器连接的硬件接口
    Buzzer_Init();

    //Initialize the hardware interface connected to the enable switch
    //初始化与使能开关连接的硬件接口
    Enable_Pin();

    //Initialize the hardware interface connected to the OLED display
    //初始化与OLED显示屏连接的硬件接口
    OLED_Init();

    //Initialize the hardware interface connected to the user's key
    //初始化与用户按键连接的硬件接口
    KEY_Init();
		if(KEY==0)			Check=1;
		else if(KEY==1)	Check=0;
    //Serial port 1 initialization, communication baud rate 115200,
    //can be used to communicate with ROS terminal
    //串口1初始化，通信波特率115200，可用于与ROS端通信
    uart1_init(115200);

    //Serial port 2 initialization, communication baud rate 9600,
    //used to communicate with Bluetooth APP terminal
    //串口2初始化，通信波特率9600，用于与蓝牙APP端通信
    uart2_init(9600);

    //Serial port 3 is initialized and the baud rate is 115200.
    //Serial port 3 is the default port used to communicate with ROS terminal
    //串口3初始化，通信波特率115200，串口3为默认用于与ROS端通信的串口
    uart3_init(115200);
//	uart3_init(230400);
	
    //Serial port 5 initialization, communication baud rate 115200,
    //can be used to communicate with ROS terminal
    //串口5初始化，通信波特率115200，可用于与ROS端通信
    uart5_init(115200);

    //ADC pin initialization, used to read the battery voltage and potentiometer gear,
    //potentiometer gear determines the car after the boot of the car model
    //ADC引脚初始化，用于读取电池电压与电位器档位，电位器档位决定小车开机后的小车适配型号
    Adc_Init();
    Adc_POWER_Init();

    //Initialize the CAN communication interface
    //CAN通信接口初始化
    CAN1_Mode_Init(1,7,6,3,0);

    //According to the tap position of the potentiometer, determine which type of car needs to be matched,
    //and then initialize the corresponding parameters
    //根据电位器的档位判断需要适配的是哪一种型号的小车，然后进行对应的参数初始化
    Robot_Select();

    //Encoder A is initialized to read the real time speed of motor C
    //编码器A初始化，用于读取电机C的实时速度
    Encoder_Init_TIM2();
    //Encoder B is initialized to read the real time speed of motor D
    //编码器B初始化，用于读取电机D的实时速度
    Encoder_Init_TIM3();
    //Encoder C is initialized to read the real time speed of motor B
    //编码器C初始化，用于读取电机B的实时速度
    Encoder_Init_TIM4();
    //Encoder D is initialized to read the real time speed of motor A
    //编码器D初始化，用于读取电机A的实时速度
    Encoder_Init_TIM5();

    //定时器12用作舵机的PWM接口
    TIM12_SERVO_Init(9999,84-1);  //APB1的时钟频率为84M , 频率=84M/((9999+1)*(83+1))=100Hz

    //普通小车默认定时器8用作航模接口
    // TIM8_SERVO_Init(9999,168-1);//APB2的时钟频率为168M , 频率=168M/((9999+1)*(167+1))=100Hz
    //Initialize the model remote control interface
    //初始化航模遥控接口
    TIM8_Cap_Init(9999,168-1);  //高级定时器TIM8的时钟频率为168M

    //Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
    //初始化电机速度控制以及，用于控制电机速度，PWM频率10KHZ
    //APB2时钟频率为168M，满PWM为16799，频率=168M/((16799+1)*(0+1))=10k
    TIM1_PWM_Init(16799,0);
    TIM9_PWM_Init(16799,0);
    TIM10_PWM_Init(16799,0);
    TIM11_PWM_Init(16799,0);

    //采集1次电池电压作为电压平滑滤波的初始化参考值
    base_vol = Get_battery_volt();

    //访问是否存在自动回充装备
    delay_ms(100);
    Find_Charging_HardWare();
	

}

void Find_Charging_HardWare(void)
{
    u8 tmpdata[8]= {1,1,1,1,1,1,1,1};
    CAN1_Send_EXTid_Num(0x12345678,tmpdata);
}

