#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "RP.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "string.h"

int16_t ExtractSpeed(void);

/*电机测试*/
/*下载此段程序后，按下K1，电机速度增加，按下K2，电机速度减小，按下K3，电机停止*/
uint8_t KeyNum;
int16_t PWM;

/*定义变量*/
float Target=0, Actual, Out;			//目标值，实际值，输出值
float Kp, Ki, Kd;					//比例项，积分项，微分项的权重
float Error0, Error1, Error2;		//本次误差，上次误差，上上次误差


int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	Key_Init();			//非阻塞式按键初始化
	Motor_Init();		//电机初始化
	Encoder_Init();		//编码器初始化
	Serial_Init();		//串口初始化，波特率9600
	
	Timer_Init();		//定时器初始化，定时中断时间1ms
	
	/*OLED打印一个标题*/
	OLED_Printf(0, 0, OLED_8X16, "Speed Control");
	OLED_Update();
	
	while (1)
	{
		
		
//		Motor_SetPWM(50);			//电机设定PWM，将PWM变量的值给电机
		
		Kp=0.8;
		Ki=0.3;
		Kd=0.2;
		Target=ExtractSpeed();
		
		/*OLED显示*/
		OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Kp);			//显示Kp
		OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Ki);			//显示Ki
		OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Kd);			//显示Kd
		
		OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", Target);	//显示目标值
		OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", Actual);	//显示实际值
		OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", Out);		//显示输出值
		
		OLED_Update();	//OLED更新，调用显示函数后必须调用此函数更新，否则显示的内容不会更新到OLED上
		
		Serial_Printf("%f,%f,%f\n", Target, Actual, Out);	//串口打印目标值、实际值和输出值							
		
//		OLED_Printf(0, 0, OLED_8X16, "PWM:%+04d", PWM);		//OLED显示PWM变量值
		
//		OLED_Update();				//OLED更新
	}
}

void TIM1_UP_IRQHandler(void)
{
	/*定义静态变量（默认初值为0，函数退出后保留值和存储空间）*/
	static uint16_t Count;		//用于计次分频
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
			
			/*获取实际速度值*/
			/*Encoder_Get函数，可以获取两次读取编码器的计次值增量*/
			/*此值正比于速度，所以可以表示速度，但它的单位并不是速度的标准单位*/
			/*此处每隔10ms获取一次计次值增量，电机旋转一周的计次值增量约为408*/
			/*因此如果想转换为标准单位，比如转/秒*/
			/*则可将此句代码改成Actual = Encoder_Get() / 408.0 / 0.01;*/
			Actual = Encoder_Get();
			
			/*获取本次误差和上次误差*/
			Error2 = Error1;            //获取上上次误差
			Error1 = Error0;			//获取上次误差
			Error0 = Target - Actual;	//获取本次误差，目标值减实际值，即为误差值
			
			
			/*PID计算*/
			/*使用位置式PID公式，计算得到输出值*/
			Out += Kp * (Error0-Error1) + Ki * Error0 + Kd * (Error0 - 2 * Error1 +Error2);
			
			/*输出限幅*/
			if (Out > 100) {Out = 100;}		//限制输出值最大为100
			if (Out < -100) {Out = -100;}	//限制输出值最小为100
			
			/*执行控制*/
			/*输出值给到电机PWM*/
			Motor_SetPWM(Out);
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

int16_t ExtractSpeed(void)
{
	static int16_t last=0;
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;    // 清除标志
        
        // 检查数据包格式是否正确（以@开头）
        if (Serial_RxPacket[0] != '@')
        {
            last=0;    // 格式错误，返回0
        }
        
        // 将字符串转换为整数
        uint16_t speed = 0;
        uint8_t i = 1;  // 从@后面的字符开始
        
        // 跳过符号位（+或-）
        int8_t sign = 1;
        if (Serial_RxPacket[1] == '+')
        {
            sign = 1;
            i++;
        }
        else if (Serial_RxPacket[1] == '-')
        {
            sign = -1;
            i++;
        }
        
        // 解析数字部分，直到遇到%或字符串结束
        for (; Serial_RxPacket[i] != '\0' && Serial_RxPacket[i] != '%'; i++)
        {
            if (Serial_RxPacket[i] >= '0' && Serial_RxPacket[i] <= '9')
            {
                speed = speed * 10 + (Serial_RxPacket[i] - '0');
            }
        }
        
        last=sign * speed;
    }
    return last;    // 默认返回0
}// Updated
