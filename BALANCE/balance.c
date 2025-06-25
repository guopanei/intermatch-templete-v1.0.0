/**
 * 大连民族大学 创一工作室 队内赛模板
	由轮趣科技C30D模板提供参考 标准库 FreeRTOS
 * @file balance.c
 * @author TuxMonkey (nqx_2004@qq.com)
 * @brief 电机驱动任务，核心运动控制任务
 * @date 2025-05-17
 * @version 0.1
 * 
 * 电机驱动任务，核心运动控制任务
	运动学逆解，根据三轴目标速度计算各车轮目标转速
	FreeRTOS任务，核心运动控制任务
	运动学逆解，根据三轴目标速度计算各车轮目标转速
	入口参数：X和Y、Z轴方向的目标运动速度
	
	返回值：无-------
 * Copyright (c) 2025 DLMU-CONE
 * 
 * 
 */

#include "balance.h"
#include "usbh_hid_joy.h"
#include "xunxian.h"
//int Time_count=0; //Time variable //计时变量

u32 Buzzer_count1 = 0;

#define SERVO_INIT_VALUE 1300
#define SERVO_MIN_VALUE 700
#define SERVO_MAX_VALUE 2500
hhh
#define SERVO_STEP 5
// Robot mode is wrong to detect flag bits
//机器人模式是否出错检测你好
int robot_mode_check_flag=0; 

int current_servo_value[6] = {SERVO_INIT_VALUE, SERVO_INIT_VALUE, SERVO_INIT_VALUE, SERVO_INIT_VALUE, SERVO_INIT_VALUE,SERVO_INIT_VALUE}; // Current servo value //当前舵机值

volatile uint32_t systick_cnt = 0;        // 系统滴答计数器


short test_num;
u8 command_lost_count=0;//

Encoder OriginalEncoder; //Encoder raw data //编码器原始数据     

//========== PWM清除使用变量 ==========//
u8 start_check_flag = 0;//标记是否需要清空PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //标记开始清除PWM
u8 clear_done_once = 0; //清除完成标志位
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
void Track_Line(void);
volatile uint8_t AutoTrackMode = 0;
/*------------------------------------*/


/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
	float amplitude=4.0; //Wheel target speed limit //车轮目标速度限幅

	
	Vy=target_limit_float(Vy,-amplitude,amplitude);
	Vz=target_limit_float(Vz,-amplitude,amplitude);
	
	//Speed smoothing is enabled when moving the omnidirectional trolley
	//全向移动小车才开启速度平滑处理
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
	{
		if(Allow_Recharge==0)
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理
		else
			smooth_control.VX=Vx,     
			smooth_control.VY=Vy,
			smooth_control.VZ=Vz;

		//Get the smoothed data 
		//获取平滑处理后的数据			
		Vx=smooth_control.VX;     
		Vy=smooth_control.VY;
		Vz=smooth_control.VZ;
	}
		
	//Mecanum wheel car
	//麦克纳姆轮小车
	if (Car_Mode==Mec_Car||Car_Mode==Mec_Car_V550) 
	{
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);

		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
	} 
		
	//Omni car
	//全向轮小车
	else if (Car_Mode==Omni_Car) 
	{
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
		MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;

		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=0;	//Out of use //没有使用到
	}
		
	//Ackermann structure car
	//阿克曼小车
	else if (Car_Mode==Akm_Car) 
	{
		//Ackerman car specific related variables //阿克曼小车专用相关变量
		float R, Ratio=636.56, AngleR, Angle_Servo;
		
		// For Ackerman small car, Vz represents the front wheel steering Angle
		//对于阿克曼小车Vz代表右前轮转向角度
		AngleR=Vz;
		R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
		
		// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
		//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
		AngleR=target_limit_float(AngleR,-0.49f,0.32f);
		
		//Inverse kinematics //运动学逆解
		if(AngleR!=0)
		{
			MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
			MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;			
		}
		else 
		{
			MOTOR_A.Target = Vx;
			MOTOR_B.Target = Vx;
		}
		// The PWM value of the servo controls the steering Angle of the front wheel
		//舵机PWM值，舵机控制前轮转向角度
		Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
		Servo=SERVO_INIT + (Angle_Servo - 1.572f)*Ratio;

		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=0; //Out of use //没有使用到
		MOTOR_D.Target=0; //Out of use //没有使用到
		Servo=target_limit_int(Servo,800,2200);	//Servo PWM value limit //舵机PWM值限幅
	}
		
	//Differential car
	//差速小车
	else if (Car_Mode==Diff_Car) 
	{
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //计算出左轮的目标速度
		MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //计算出右轮的目标速度

		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=0; //Out of use //没有使用到
		MOTOR_D.Target=0; //Out of use //没有使用到
	}
		
	//FourWheel car
	//四驱车
	else if(Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550) 
	{	
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
		MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
		MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
		MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
				
		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 	
	}

	//Tank Car
	//履带车
	else if (Car_Mode==Tank_Car) 
	{
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //计算出左轮的目标速度
		MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //计算出右轮的目标速度

		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=0; //Out of use //没有使用到
		MOTOR_D.Target=0; //Out of use //没有使用到
	}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/

typedef enum {
    TURN_NONE  = 0,
    TURN_LEFT  = 1,
    TURN_RIGHT = 2
} PresetTurnType; // T型路口预设转向
typedef enum {
    TRACK_MODE_OFF = 0,
    TRACK_MODE_ON  = 1
} TrackModeType;// 循迹模式标志
TrackModeType track_mode = TRACK_MODE_OFF;
PresetTurnType preset_turn = TURN_NONE; 
#define LINE_PID_KP  0.8f   // 比例系数
#define LINE_PID_KI  0.05f  // 积分系数
#define LINE_PID_KD  0.2f   // 微分系数
// 限幅函数：将 value 限制在 [min_val, max_val] 之间
float constrain(float value, float min_val, float max_val);
float constrain(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
void Balance_task(void *pvParameters)
{ 
	u32 lastWakeTime = getSysTickCnt();

    while(1)
    {	
		// This task runs at a frequency of 100Hz (10ms control once)
		//此任务以100Hz的频率运行（10ms控制一次）
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 

		//Time count is no longer needed after 30 seconds
		//时间计数，30秒后不再需要
		if(SysVal.Time_count<3000) SysVal.Time_count++;
			Buzzer_count1++;

		//Get the encoder data, that is, the real time wheel speed, 
		//and convert to transposition international units
		//获取编码器数据，即车轮实时速度，并转换位国际单位
		Get_Velocity_Form_Encoder();   
		

			//Click the user button to update the gyroscope zero
			//单击用户按键更新陀螺仪零点
			Key(); 
			
// 模式切换
if (Get_PS2_KEY(SELECT_KEY)) {
track_mode = (TrackModeType)(!track_mode);  
    preset_turn = TURN_NONE; // 重置预设转向
}

if( Allow_Recharge==1 )
    if( Get_Charging_HardWare==0 ) Allow_Recharge=0,Find_Charging_HardWare();
// 修复后的自检模式代码
if(Check==0) //If self-check mode is not enabled
{
      if (track_mode && (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550)) {
            
            static float turn_smoothing = 0.8f;
            static float last_turn_speed = 0;  
            static uint32_t lost_timer = 0;    // 【优化】改为静态变量
            
            // 基础前进速度计算（关联遥控器速度，更安全）
						static float RC_Velocity = 0.0f; // 需实现该函数
            float line_forward_speed = -RC_Velocity * 0.6f;
            float base_forward = line_forward_speed / 1000.0f;
            base_forward = constrain(base_forward, -1.0f, 1.0f); // 限幅保护

            // 4. OpenMV 数据驱动
            if (openmv_line_detected) { 
                lost_timer = 0; // 重置丢失计时
                
                // 转向值映射（根据实际通信协议调整）
                float turn_speed = (float)openmv_turn_value / 50.0f;
                turn_speed = constrain(turn_speed, -2.0f, 2.0f); // 限幅
                
                // 转向平滑过渡（减少电机冲击）
                turn_speed = turn_smoothing * turn_speed + 
                          (1.0f - turn_smoothing) * last_turn_speed;
                last_turn_speed = turn_speed;

                // 赋值运动分量（麦克纳姆轮坐标系）
                Move_X = base_forward;
                Move_Z = turn_speed;
                Move_Y = 0; // 麦克纳姆轮 Y 轴通常为横向移动
            } 
            // 5. 丢线处理逻辑
            else { 
                if (lost_timer == 0) {
                    lost_timer = systick_cnt; // 记录丢线时刻
                }
                // 低速续行寻找线路
                Move_X = base_forward * 0.5f;
                Move_Z = last_turn_speed; 
                Move_Y = 0;

                // 超时强制停止（200ms阈值，可调整）
                if (systick_cnt - lost_timer > 200) { 
                    Move_X = 0;
                    Move_Z = 0;
                    last_turn_speed = 0;
                }
            }

            // 6. T型路口预设转向（独立逻辑层）
            if (preset_turn != TURN_NONE) { 
                if (preset_turn == TURN_LEFT) {
                    Move_Z = 1.5f;  // 左转幅度（需与PID配合）
                } else if (preset_turn == TURN_RIGHT) {
                    Move_Z = -1.5f; // 右转幅度
                }
                preset_turn = TURN_NONE; // 单次触发
            }

            // 7. 电机驱动（建议增加安全检查）
            if (fabs(Move_X) < 0.01f && fabs(Move_Z) < 0.01f) {
                // 防止微小值导致电机抖动
                Move_X = 0;
                Move_Z = 0;
            }
            Drive_Motor(Move_X, Move_Y, Move_Z); 
        }
    

else {

			if(Allow_Recharge==1)
			{
				if(Get_Charging_HardWare==1)
				{   //存在回充装备时，对回充装备的状态进行检测
					charger_check++;
					if( charger_check>RATE_100_HZ) charger_check=RATE_100_HZ+1,Allow_Recharge=0,RED_STATE=0,Recharge_Red_Move_X = 0,Recharge_Red_Move_Y = 0,Recharge_Red_Move_Z = 0;
				}
				//如果开启了导航回充，同时没有接收到红外信号，接收来自上位机的的回充控制命令
				if      (nav_walk==1 && RED_STATE==0) Drive_Motor(Recharge_UP_Move_X,0,Recharge_UP_Move_Z);
				//接收到了红外信号，接收来自回充装备的回充控制命令
				else if (RED_STATE!=0) nav_walk = 0,Drive_Motor(Recharge_Red_Move_X,0,Recharge_Red_Move_Z);
				//防止没有红外信号时小车运动
				if (nav_walk==0&&RED_STATE==0) Drive_Motor(0,0,0);
			}
			else
			{			
				if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //处理APP遥控命令
				else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //处理航模遥控命令
				else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //处理PS2手柄控制命令

				//CAN, Usart 1, Usart 3, Uart5 control can directly get the three axis target speed, 
				//without additional processing
				//CAN、串口1、串口3(ROS)、串口5控制直接得到三轴目标速度，无须额外处理
				else                      Drive_Motor(Move_X, Move_Y, Move_Z);
			}
		}


			

			//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
			//and the software failure flag is 0
			//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
			if(Turn_Off(Voltage)==0||(Allow_Recharge&&EN&&!Flag_Stop)) 
			{ 			
				//Speed closed-loop control to calculate the PWM value of each motor, 
				//PWM represents the actual wheel speed					 
				//速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
				MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
				MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
				MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
				MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);

				Limit_Pwm(16700);

				//检测是否需要清除PWM并自动执行清理
				auto_pwm_clear();
				
				//Set different PWM control polarity according to different car models
				//根据不同小车型号设置不同的PWM控制极性
				switch(Car_Mode)
				{
					case Mec_Car:case Mec_Car_V550:
						Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Mecanum wheel car       //麦克纳姆轮小车
					case Omni_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Omni car                //全向轮小车
					case Akm_Car:       Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Ackermann structure car //阿克曼小车
					case Diff_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Differential car        //两轮差速小车
					case FourWheel_Car:case FourWheel_Car_V550:
						Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //FourWheel car           //四驱车 
					case Tank_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Tank Car                //履带车
				}
			}
			//If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
			//如果Turn_Off(Voltage)返回值为1，不允许控制小车进行运动，PWM值设置为0
			else	Set_Pwm(0,0,0,0,0); 
		}	
		else							//用户自检代码
			{
				if(Proc_Flag==3)						//自检电机
				{
					 if(check_time_count_motor_forward>0)
					 {	 
						 check_time_count_motor_forward--;
						 Full_rotation=16799;
					 }
					 else if(check_time_count_motor_retreat>0) 
					 {	 
							check_time_count_motor_retreat--;
						 Full_rotation=-16799;
					 }		

					 switch(Car_Mode)
					 {
						    case Mec_Car:case Mec_Car_V550: 
							    Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0    ); break; //Mecanum wheel car       //麦克纳姆轮小车
							case Omni_Car:      Set_Pwm(-Full_rotation,  Full_rotation, -Full_rotation, Full_rotation, 0    ); break; //Omni car                //全向轮小车
							case Akm_Car:       Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Ackermann structure car //阿克曼小车
							case Diff_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Differential car        //两轮差速小车
							case FourWheel_Car:case FourWheel_Car_V550:
								Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0    ); break; //FourWheel car           //四驱车 
							case Tank_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Tank Car                //履带车
					 } 
					 if(!(check_time_count_motor_retreat>0) && !(check_time_count_motor_forward>0))
					 {	 
						 Set_Pwm(0,0,0,0,0);		 
					 }
				}
				if(Proc_Flag==4)		Set_Pwm(0,0,0,0,0);
				if(Proc_Flag==6)		TIM8_SERVO_Init(9999,168-1);					//六路舵机
				if(Proc_Flag==7)																					//控制舵机
				{
					if(servo_direction[0]==0&&Servo_Count[0]<2500) Servo_Count[0]=Servo_Count[0]+5;
				 if(servo_direction[0]==0&&Servo_Count[0]>=2500) servo_direction[0]=1;
				 if(Servo_Count[0]>500&&servo_direction[0]==1)  Servo_Count[0]=Servo_Count[0]-5;
				 if(Servo_Count[0]<=500&&servo_direction[0]==1)  Servo_Count[0]=500,servo_direction[0] = 2;
				 TIM12->CCR2=Servo_Count[0];
				 
				}
				if(Proc_Flag==8)
				{
					if(servo_direction[0]!=2)					Servo_Count[0]=500,TIM12->CCR2=Servo_Count[0];
					if(servo_direction[1]==0&&Servo_Count[1]<2500) Servo_Count[1]=Servo_Count[1]+5;
				 if(servo_direction[1]==0&&Servo_Count[1]>=2500) servo_direction[1]=1;
				 if(Servo_Count[1]>500&&servo_direction[1]==1)  Servo_Count[1]=Servo_Count[1]-5;
				 if(Servo_Count[1]<=500&&servo_direction[1]==1)  Servo_Count[1]=500,servo_direction[1] = 2;
					TIM12->CCR1=Servo_Count[1];
				}
				if(Proc_Flag==9)
				{
					if(servo_direction[1]!=2)					Servo_Count[1]=500,TIM12->CCR1=Servo_Count[1];
					if(servo_direction[2]==0&&Servo_Count[2]<2500) Servo_Count[2]=Servo_Count[2]+5;
				 if(servo_direction[2]==0&&Servo_Count[2]>=2500) servo_direction[2]=1;
				 if(Servo_Count[2]>500&&servo_direction[2]==1)  Servo_Count[2]=Servo_Count[2]-5;
				 if(Servo_Count[2]<=500&&servo_direction[2]==1)  Servo_Count[2]=500,servo_direction[2] = 2;
					TIM8->CCR4=Servo_Count[2];
				}
				if(Proc_Flag==10)
				{
					if(servo_direction[2]!=2)					Servo_Count[2]=500,TIM8->CCR4=Servo_Count[2];
					if(servo_direction[3]==0&&Servo_Count[3]<2500) Servo_Count[3]=Servo_Count[3]+5;
				 if(servo_direction[3]==0&&Servo_Count[3]>=2500) servo_direction[3]=1;
				 if(Servo_Count[3]>500&&servo_direction[3]==1)  Servo_Count[3]=Servo_Count[3]-5;
				 if(Servo_Count[3]<=500&&servo_direction[3]==1)  Servo_Count[3]=500,servo_direction[3] = 2;
					TIM8->CCR3=Servo_Count[3];
				}
				if(Proc_Flag==11)
				{
					if(servo_direction[3]!=2)					Servo_Count[3]=500,TIM8->CCR3=Servo_Count[3];
					if(servo_direction[4]==0&&Servo_Count[4]<2500) Servo_Count[4]=Servo_Count[4]+5;
				 if(servo_direction[4]==0&&Servo_Count[4]>=2500) servo_direction[4]=1;
				 if(Servo_Count[4]>500&&servo_direction[4]==1)  Servo_Count[4]=Servo_Count[4]-5;
				 if(Servo_Count[4]<=500&&servo_direction[4]==1)  Servo_Count[4]=500,servo_direction[4] = 2;
					TIM8->CCR2=Servo_Count[4];
				}
				if(Proc_Flag==12)
				{
					if(servo_direction[4]!=2)					Servo_Count[4]=500,TIM8->CCR2=Servo_Count[4];
					if(servo_direction[5]==0&&Servo_Count[5]<2500) Servo_Count[5]=Servo_Count[5]+5;
				 if(servo_direction[5]==0&&Servo_Count[5]>=2500) servo_direction[5]=1;
				 if(Servo_Count[5]>500&&servo_direction[5]==1)  Servo_Count[5]=Servo_Count[5]-5;
				 if(Servo_Count[5]<=500&&servo_direction[5]==1)  Servo_Count[5]=500,servo_direction[5] = 2;
					TIM8->CCR1=Servo_Count[5];
				}
				
				if(Proc_Flag==13)																	//
				{
					servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
					Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
					 TIM8->CCR1=Servo_Count[5];
					 TIM8->CCR2=Servo_Count[4];
					 TIM8->CCR3=Servo_Count[3];
					 TIM8->CCR4=Servo_Count[2];
					 TIM12->CCR1=Servo_Count[1];
					 TIM12->CCR2=Servo_Count[0];
				}
				if(Proc_Flag==14)																	//蜂鸣器间隔1s响一次
				{
					if((Buzzer_count1/100)%2)			Buzzer = 1;
					else													Buzzer = 0;
				}
				if(Proc_Flag==15)			Buzzer = 0;
//				if(Proc_Flag==17)																	//向APP发送WHEELTEC
//				{
//					if(uart2_send_flag==1)
//					{
//						USART2_Return();
//						uart2_send_flag = 0;
//						app_count = 0;
//					}
//				}
				if(Proc_Flag==19)
				{
					if(uart3_send_flag==1)
					{
						USART3_Return();
						uart3_send_flag = 0;
						message_count = 0;
					}
				}
			}
	}  
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a;
	else 	            PWMA2=16799,PWMA1=16799-motor_a;
	
	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
	else 	            PWMC2=16799,PWMC1=16799-motor_c;
	
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            PWMD2=16799,PWMD1=16799-motor_d;
	
	//Servo control
	//舵机控制
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC1=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	
	//清除PWM标志位，该位为1时代表需要清除PWM
	if( start_clear ) 
	{
		//PWM逐渐递减的方式清除，减缓小车由于电机释放而造成轻微移动的影响
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		//若清除完毕，则标记标志位，4个电机分别用4个bit表示
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<0;
		else clear_state &= ~(1<<0);
	}
	
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<1;
		else clear_state &= ~(1<<1);
	}
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	
	if(Car_Mode==Diff_Car || Car_Mode==Akm_Car || Car_Mode==Tank_Car) Pwm = 0;
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<2;
		else clear_state &= ~(1<<2);
	}
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	
	if(Car_Mode==Diff_Car || Car_Mode==Akm_Car || Car_Mode==Tank_Car || Car_Mode==Omni_Car ) Pwm = 0;
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<3;
		else clear_state &= ~(1<<3);
		
		//4个电机均清除完毕，则关闭清除任务
		if( (clear_state&0xff)==0x0f ) start_clear = 0,clear_done_once=1,clear_state=0;
	}
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		               Move_Z=0;                       //stop           //停止
	 }
	}	
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
	}
	
	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/

// ===== 循迹控制函数 =====
//void Track_Line(void)
// {
//Sensor_Init();
//    // 获取传感器状态

//	  SensorState[0] = Sensor0_Get_State();
//     SensorState[1] = Sensor3_Get_State();
//    SensorState[2] = Sensor1_Get_State();
//    SensorState[3] = Sensor2_Get_State();
//}

void PS2_control(void)
{ 

  int LX,LY,RY,RX;
    int Threshold=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
    static u8 acc_dec_filter = 0;	
	static u8 PS2_KEY_Last_State = 0;	

    //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
    //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
    LY=(PS2_LX-128);  
    LX=(PS2_LY-128); 
    RY=(PS2_RX-128);
    RX=(PS2_RY-128); 

    //Ignore small movements of the joystick //忽略摇杆小幅度动作
    if(LX>-Threshold&&LX<Threshold)LX=0; 
    if(LY>-Threshold&&LY<Threshold)LY=0; 
    if(RY>-Threshold&&RY<Threshold)RY=0; 
    if(RX>-Threshold&&RX<Threshold)RX=0; 

    if(++acc_dec_filter==15)
    {
        acc_dec_filter=0;
        if (PS2_KEY==11)    RC_Velocity+=50;  //To accelerate//加速
        else if(PS2_KEY==9) RC_Velocity-=1000;  //To slow down //减速	
    }

    if(RC_Velocity<0)   RC_Velocity=0;

	  static u8 is_initialized = 0;
  if (!is_initialized) {
       // 仅初始化一次
      TIM12_SERVO_Init(19999,83); // 仅初始化一次
      is_initialized = 1;
  }

      if (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550) {
        u8 current_key_state = Get_PS2_KEY(SELECT_KEY); // 当前按键状态
        // 检测上升沿（仅在按键按下瞬间触发）
        if (current_key_state && !PS2_KEY_Last_State) { 
            AutoTrackMode = !AutoTrackMode; // 取反切换模式
            // 切换时重置速度，避免模式切换瞬间的突变
            Move_X = 0;
            Move_Y = 0;
            Move_Z = 0;
        }
        PS2_KEY_Last_State = current_key_state; // 更新上一次状态（关键修复：确保释放后状态更新）
    }

    // ====================== 非巡线模式下的手柄控制逻辑 ======================
    if (!AutoTrackMode) {
    //Handle PS2 controller control commands
    //对PS2手柄控制命令进行处理
    Move_X=LX*RC_Velocity/80; // 左摇杆 Y 轴控制前后
    Move_Z=LY*(PI/2)/80;     // 左摇杆 X 轴控制旋转
    Move_Y=RY*RC_Velocity/120; // 右摇杆 X 轴控制左右平移

    //Z-axis data conversion //Z轴数据转化
    if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
    {
        Move_Z=Move_Z*RC_Velocity/500;
    }	
    else if(Car_Mode==Akm_Car)
    {
        //Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
        //阿克曼结构小车转换为前轮转向角度
        Move_Z=Move_Z*4/9;
    }
    else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
    {
        if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
        Move_Z=Move_Z*RC_Velocity/250;
    }	

    //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
    Move_X=Move_X/1000;        
    Move_Y=Move_Y/1000;    
    Move_Z=Move_Z;

    //Control target value is obtained and kinematics analysis is performed
    //得到控制目标值，进行运动学分析
    Drive_Motor(Move_X,Move_Y,Move_Z);
}       
   
    // 舵机控制部分
	 TIM8_SERVO_Init(19999,168-1);
  if (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550) {
      if (Get_PS2_KEY(RT_GREEN)) {
            if (current_servo_value[1] + SERVO_STEP <= SERVO_MAX_VALUE) {
                current_servo_value[1] += SERVO_STEP;
				 
            }
        }
        // 检查 RT_BLUE 按键是否按下，按下则减少舵机角度
        if (Get_PS2_KEY(RT_BLUE)) {
            if (current_servo_value[1] - SERVO_STEP >= SERVO_MIN_VALUE) {
                current_servo_value[1] -= SERVO_STEP;
				  
            }
        }

		if (Get_PS2_KEY(RT_PINK)) {
            if (current_servo_value[2] + SERVO_STEP <= SERVO_MAX_VALUE) {
                current_servo_value[2] += SERVO_STEP;
				  
            }
        }
        // 检查 RT_BLUE 按键是否按下，按下则减少舵机角度
        if (Get_PS2_KEY(RT_RED)) {
            if (current_servo_value[2] - SERVO_STEP >= SERVO_MIN_VALUE) {
                current_servo_value[2] -= SERVO_STEP;
				
            }
        }
		if (Get_PS2_KEY(LF_UP)) {
            if (current_servo_value[3] + SERVO_STEP <= SERVO_MAX_VALUE) {
                current_servo_value[3] += SERVO_STEP;
				 
            }
        }
        // 检查 RT_BLUE 按键是否按下，按下则减少舵机角度
        if (Get_PS2_KEY(LF_DOWN)) {
            if (current_servo_value[3] >1075) {

                current_servo_value[3] =1075;
				  
            }
        }
		if (Get_PS2_KEY(LF_RIGHT)) {
            if (current_servo_value[4] >950) {
                current_servo_value[4] =950;
				 
            }
			else  current_servo_value[4] =1550;
			 vTaskDelay(2);
        }
        // 检查 RT_BLUE 按键是否按下，按下则减少舵机角度
        if (Get_PS2_KEY(LF_LEFT)) {
            if (current_servo_value[4] - SERVO_STEP >= SERVO_MIN_VALUE) {
                current_servo_value[4] -= SERVO_STEP;
				  
            }
        }
		if (Get_PS2_KEY(R1_KEY)) {
            if (current_servo_value[0] >750) {
                current_servo_value[0] =750;
				 
            }
			else  current_servo_value[0] =1450;
			 vTaskDelay(2);
        }
        // 检查 RT_BLUE 按键是否按下，按下则减少舵机角度
        if (Get_PS2_KEY(R2_KEY)) {
            if (current_servo_value[0] - SERVO_STEP >= SERVO_MIN_VALUE) {
                current_servo_value[0] -= SERVO_STEP;
				  
            }
        }
		if (Get_PS2_KEY(L1_KEY)){
			// 修复：使用循环为数组元素赋值
			for (int i = 5; i < 6; i++) {
				current_servo_value[i] = SERVO_INIT_VALUE;
			}
			
				current_servo_value[2] = 700;
				current_servo_value[1] = 2085;
			    current_servo_value[0] =1450;
			current_servo_value[4] =950;
				
		}
	

        // 将更新后的舵机角度值写入定时器寄存器
       if (current_servo_value[1] >= SERVO_MIN_VALUE && current_servo_value[1] <= SERVO_MAX_VALUE) {
            TIM8->CCR4 = current_servo_value[1];
        }
        if (current_servo_value[2] >= SERVO_MIN_VALUE && current_servo_value[2] <= SERVO_MAX_VALUE) {
            TIM8->CCR3 = current_servo_value[2];
        }
        if (current_servo_value[3] >= SERVO_MIN_VALUE && current_servo_value[3] <= SERVO_MAX_VALUE) {
            TIM8->CCR2 = current_servo_value[3];
        }
        if (current_servo_value[4] >= SERVO_MIN_VALUE && current_servo_value[4] <= SERVO_MAX_VALUE) {
            TIM8->CCR1 = current_servo_value[4];	
        }
		   if (current_servo_value[0] >= SERVO_MIN_VALUE && current_servo_value[0] <= SERVO_MAX_VALUE) {
            TIM12->CCR1 = current_servo_value[0];	
        }
    }
	
   
}

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    static u8 thrice=100; 
    int Threshold=100; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作

	  //limiter //限幅
    int LX,LY,RY,RX,Remote_RCvelocity; 
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

	  // Front and back direction of left rocker. Control forward and backward.
	  //左摇杆前后方向。控制前进后退。
    LX=Remoter_Ch2-1500; 
	
	  //Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	  //Ackerman trolleys use this channel as a PWM output to control the steering gear
	  //左摇杆左右方向。控制左右移动。麦轮全向轮才会使用到改通道。阿克曼小车使用该通道作为PWM输出控制舵机
    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//右摇杆前后方向。油门/加减速。
	  RX=Remoter_Ch3-1500;

    //Right stick left and right. To control the rotation. 
		//右摇杆左右方向。控制自转。
    RY=Remoter_Ch1-1500; 

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RX>-Threshold&&RX<Threshold)RX=0;
	  if(RY>-Threshold&&RY<Threshold)RY=0;
		
		//Throttle related //油门相关
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//对航模遥控控制命令进行处理
    Move_X= LX*Remote_RCvelocity/500; 
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/2)/500;      
			 
		//Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
		{
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //阿克曼结构小车转换为前轮转向角度
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}
		
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;       
    Move_Y=Move_Y/1000;      
		Move_Z=Move_Z;
		
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
				
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
    u8 tmp;
    //传入任务的频率
    tmp=KEY_Scan(RATE_100_HZ,0);
		if(Check==0)
		{
    //单击 或 手柄同时按下两边的下扳机，开启自动回充
    if(tmp==single_click || ( Get_PS2_KEY(L2_KEY) && Get_PS2_KEY(R2_KEY) ) )
	{
		Allow_Recharge=!Allow_Recharge;
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}		

    //双击 或 手柄同时按下两边的摇杆,更新陀螺仪
    else if(tmp==double_click || ( Get_PS2_KEY(LF_ROCKER_KEY) && Get_PS2_KEY(RT_ROCKER_KEY) )  ) 
	{
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}

    //长按 切换页面
    else if(tmp==long_click )
    {
        oled_refresh_flag=1;
        oled_page++;
        if(oled_page>OLED_MAX_Page-1) oled_page=0;
    }
	}
		else if(Check==1)
		{
			if(tmp==single_click)		
			{
				Proc_Flag++;
				if(Proc_Flag==21)			
				{
					Check = 0;
					Buzzer = 0;
					Proc_Flag = 0;
					check_time_count_motor_forward=300;
					check_time_count_motor_retreat=500;
					Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
					servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
					TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE); 
				}
			}
			else if(tmp==double_click)
			{
				Check = 0;
				Buzzer = 0;
				Proc_Flag = 0;
				check_time_count_motor_forward=300;
				check_time_count_motor_retreat=500;
				Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
				servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
				TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE); 
			}
		}
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	//test_num=OriginalEncoder.B;
	
	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
		switch(Car_Mode)
		{
			case Mec_Car:case Mec_Car_V550:
			case FourWheel_Car:case FourWheel_Car_V550:
                Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Akm_Car:case Diff_Car:case Tank_Car:
				Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Omni_Car:    
				Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break;
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=5;
	
	if(PS2_ON_Flag)
	{
		step=5;
	}
	else
	{
		step=0.01;
	}
	
	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

u32 int_abs(int a)
{
	u32 temp;
	if(a<0) temp=-a;
	else temp = a;
	return temp;
}

/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.Out of service
Input   : none
Output  : none
函数功能：防止电位器选错模式，导致初始化出错引发电机乱转。已停止使用
入口参数：无
返回  值：无
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//如果连续6次接近满幅输出，判断为电机乱转，让电机失能	
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  
}

//PWM消除函数
void auto_pwm_clear(void)
{
	//小车姿态简易判断
	float y_accle = (float)(imu.accel.y/1671.84f);//Y轴加速度实际值
	float z_accle = (float)(imu.accel.z/1671.84f);//Z轴加速度实际值
	float diff;
	
	//计算Y、Z加速度融合值，该值越接近9.8，表示小车姿态越水平
	if( y_accle > 0 ) diff  = z_accle - y_accle;
	else diff  = z_accle + y_accle;
	
//	debug_show_diff = diff;
	
	//PWM消除检测
	if( MOTOR_A.Target !=0.0f || MOTOR_B.Target != 0.0f || MOTOR_C.Target != 0.0f || MOTOR_D.Target != 0.0f )
	{
		start_check_flag = 1;//标记需要清空PWM
		wait_clear_times = 0;//复位清空计时
		start_clear = 0;     //复位清除标志
		
		
		//运动时斜坡检测的数据复位
		clear_done_once = 0;
		clear_again_times=0;
	}
	else //当目标速度由非0变0时，开始计时 2.5 秒，若小车不在斜坡状态下，清空pwm
	{
		if( start_check_flag==1 )
		{
			wait_clear_times++;
			if( wait_clear_times >= 250 )
			{
				//小车在水平面上时才标记清空pwm，防止小车在斜坡上运动出现溜坡
				if( diff > 8.8f )	start_clear = 1,clear_state = 0;//开启清除pwm
				else clear_done_once = 1;//小车在斜坡上，标记已完成清除
				
				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	//完成了清除后，若出现推车行为，pwm积累一定数值后将在10秒后再次清空
	if( clear_done_once )
	{
		//小车接近于水平面时才作积累消除，防止小车在斜坡上溜车
		if( diff > 8.8f )
		{
			//完成清除后pwm再次积累，重新清除
			if( int_abs(MOTOR_A.Motor_Pwm)>300 || int_abs(MOTOR_B.Motor_Pwm)>300 || int_abs(MOTOR_C.Motor_Pwm)>300 || int_abs(MOTOR_D.Motor_Pwm)>300 )
			{
				clear_again_times++;
				if( clear_again_times>1000 )
				{
					clear_done_once = 0;
					start_clear = 1;//开启清除pwm
					clear_state = 0;
				}
			}
			else
			{
				clear_again_times = 0;
			}
		}
		else
		{
			clear_again_times = 0;
		}

	}
}

