/********************************************************
 * PID基本示例
 * 读取模拟输入0以控制模拟PWM输出3
 ********************************************************/

#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
// 定义我们将要连接到的变量
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
// 指定链接和初始调优参数
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  // 初始化我们链接到的变量
  Input = analogRead(PIN_INPUT);
  Setpoint = 100;

  //turn the PID on
  // 打开PID
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(PIN_INPUT);
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}

void main()
{
  setup();
  while (true)
  {
    loop();
  }
}
