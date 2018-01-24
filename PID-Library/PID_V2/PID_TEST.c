#include "PID.h"
#include <stdio.h>
/********************************************************
 * PID基本示例
 * 读取模拟输入0以控制模拟PWM输出3
 ********************************************************/

#define PIN_INPUT 0
#define PIN_OUTPUT 3
float data[50] =
    {
        0.3192, 0.2835, 0.4374, 0.3132, 0.3018, 0.3658, 0.3758, 0.2931, 0.2798, 0.3994, 0.3157, 0.2807, 0.4234, 0.3192, 0.2528, 0.4041, 0.3025, 0.2872, 0.3821, 0.2884, 0.2933, 0.3656, 0.2878, 0.3126, 0.4165, 0.3811, 0.3470, 0.3098, 0.2721, 0.2826, 0.3283, 0.2931, 0.4071, 0.2764, 0.4377, 0.0509, 0.0733, 0.1111, 0.1502, 0.2947, 0.3388, 0.3166, 0.3758, 0.3325, 0.3809, 0.3304, 0.3144, 0.2729, 0.2785};
//Define Variables we'll be connecting to
// 定义我们将要连接到的变量
double Setpoint, Input, Output;
PID pid;
PID_Config pidcfg = {
    &Input,
    &Output,
    &Setpoint,
    1,
    1,
    1,
    P_ON_M,
    DIRECT,
    100,
    1, 2,
    AUTOMATIC};
int i = 0;
//Specify the links and initial tuning parameters
// 指定链接和初始调优参数
void setup()
{
    PID_Init(&pid, &pidcfg);
    //initialize the variables we're linked to
    // 初始化我们链接到的变量

    Setpoint = 100;
    //turn the PID on
    // 打开PID
}

void loop()
{
    for (int i = 0; i < 50; i++)
    {
        Input = data[i];
        i++;
        PID_Compute(&pid);
        printf("%f,%f,%f\r\n", Setpoint, Input, Output);
    }
}

void main()
{
    setup();

    loop();
}