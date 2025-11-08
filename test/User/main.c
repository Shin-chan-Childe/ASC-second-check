#include "stm32f10x.h"                  
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "string.h"
#include <math.h>

int16_t ExtractSpeed(void);
void Speed_control(void);
void Motor_drive(void);

/*电机测试*/
uint8_t KeyNum;
int16_t PWM;

/*定义变量*/
float Target = 0, Actual, Out;           // 目标值，实际值，输出值
float Actual_left = 0;
float Target_right = 0, Actual_right = 0, Out_right = 0;
float Kp = 0.3, Ki = 0.05, Kd = 0.1;    // 优化后的速度PID参数
float Kp_pos = 0.3, Ki_pos = 0.02, Kd_pos = 0.1;  // 位置PID参数
float Error0, Error1, Error2;           // 速度误差
float Error0_right = 0, Error1_right = 0, ErrorInt_right = 0;  // 位置误差
uint8_t mode = 1;                       // 0: Motor drive, 1: Speed control

int main(void)
{
    /*模块初始化*/
    OLED_Init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    Serial_Init();
    Timer_Init();       // 10ms中断
    
    while (1)
    {
        KeyNum = Key_GetNum();
        
        // 按键切换模式
        if (KeyNum == 1)
        {
            mode = !mode;
            
            if (mode == 0)  // 切换到Motor drive模式
            {
                // 重置位置相关变量
                Actual_left = 0;
                Actual_right = 0;
                Target_right = 0;
                Error0_right = 0;
                Error1_right = 0;
                ErrorInt_right = 0;
                
                // 停止电机
                Motor_SetPWM(0);
                Motor_SetPWM_right(0);
            }
            else  // 切换到Speed control模式
            {
                // 重置速度PID变量
                Target = 0;
                Actual = 0;
                Out = 0;
                Error0 = 0;
                Error1 = 0;
                Error2 = 0;
            }
            
            Delay_ms(200);  // 防抖延时
        }
        
        // 根据模式执行不同功能
        if (mode == 0)
        {
            OLED_Printf(0, 0, OLED_8X16, "Motor Drive     ");
            Motor_drive();
        }
        else
        {
            OLED_Printf(0, 0, OLED_8X16, "Speed Control   ");
            Speed_control();
        }
        
        OLED_Update();
        Delay_ms(10);
    }
}

void TIM1_UP_IRQHandler(void)
{
    static float Filtered_Actual = 0;  // 滤波后的速度
    
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        if (mode == 1)  // Speed control模式
        {            
            // 读取原始速度并滤波
            float raw_speed = Encoder_Get();
            Filtered_Actual = 0.7 * Filtered_Actual + 0.3 * raw_speed;
            Actual = Filtered_Actual;
            
            // 更新误差
            Error2 = Error1;
            Error1 = Error0;
            Error0 = Target - Actual;
            
            // 增量式PID - 优化版本
            float deltaOut = Kp * (Error0 - Error1) + Ki * Error0 + Kd * (Error0 - 2*Error1 + Error2);
            
            // 限制单次增量，避免突变
            if (deltaOut > 8.0) deltaOut = 8.0;
            if (deltaOut < -8.0) deltaOut = -8.0;
            
            Out += deltaOut;
            
            // ========== 必须保留的输出限幅 ==========
            if (Out > 100.0) Out = 100.0;
            if (Out < -100.0) Out = -100.0;
            // =====================================
            
            // 执行控制
            Motor_SetPWM(Out);
        }
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

void Speed_control(void)
{
    Target = ExtractSpeed();
    
    /*OLED显示*/
    OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Kp);
    OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Ki);
    OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Kd);
    
    OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", Target);
    OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", Actual);
    OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", Out);
    
    Serial_Printf("%f,%f,%f\n", Target, Actual, Out);
}

void Motor_drive(void)
{
    // 读取位置增量
    Actual_left += Encoder_Get();
    Actual_right += Encoder_Get_right();
    
    Target_right = Actual_left;
    
    // 位置PID计算
    Error1_right = Error0_right;
    Error0_right = Target_right - Actual_right;
    
    // ========== 输入死区优化 ==========
    float input_deadzone = 3.0;  // 3个编码器计数
    if (fabs(Error0_right) < input_deadzone) {
        Error0_right = 0;
    }
    // ================================
    
    // 积分分离
    if (fabs(Error0_right) < 100.0)
    {
        ErrorInt_right += Error0_right;
        // 积分限幅
        if (ErrorInt_right > 500.0) ErrorInt_right = 500.0;
        if (ErrorInt_right < -500.0) ErrorInt_right = -500.0;
    }
    else
    {
        ErrorInt_right = 0;
    }
    
    // 位置PID
    Out_right = Kp_pos * Error0_right + Ki_pos * ErrorInt_right + Kd_pos * (Error0_right - Error1_right);
    
    // 输出限幅
    if (Out_right > 80.0) Out_right = 80.0;
    if (Out_right < -80.0) Out_right = -80.0;
    
    // ========== 输出死区补偿 ==========
    float output_deadzone = 12.0;  // 电机启动所需最小PWM
    if (fabs(Out_right) < output_deadzone && fabs(Error0_right) > input_deadzone) 
    {
        Out_right = (Error0_right > 0) ? output_deadzone : -output_deadzone;
    }
    // ================================
    
    // 控制右电机
    Motor_SetPWM_right(Out_right);
    
    // 显示信息
    OLED_Printf(0, 16, OLED_8X16, "L:%+06.0f", Actual_left);
    OLED_Printf(0, 32, OLED_8X16, "R:%+06.0f", Actual_right);
    OLED_Printf(0, 48, OLED_8X16, "Out:%+03.0f E:%+04.0f", Out_right, Error0_right);
    
    Serial_Printf("%f,%f,%f,%f\r\n", Actual_left, Target_right, Actual_right, Out_right);
}

int16_t ExtractSpeed(void)
{
    static int16_t last = 0;
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        
        Serial_Printf("Received: %s\r\n", Serial_RxPacket);
        
        // 直接解析符号和数字
        int8_t sign = 1;
        uint8_t start_index = 0;
        
        if (Serial_RxPacket[0] == '+')
        {
            sign = 1;
            start_index = 1;

        }
        else if (Serial_RxPacket[0] == '-')
        {
            sign = -1;
            start_index = 1;
        }
        
        // 解析数字部分
        uint16_t speed = 0;
        for (uint8_t i = start_index; Serial_RxPacket[i] != '\0'; i++)
        {
            if (Serial_RxPacket[i] >= '0' && Serial_RxPacket[i] <= '9')
            {
                speed = speed * 10 + (Serial_RxPacket[i] - '0');
            }
        }
        
        last = sign * speed;
    }
    return last;
}