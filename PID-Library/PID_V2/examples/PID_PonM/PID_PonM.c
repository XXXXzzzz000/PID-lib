/********************************************************
PID比例测量示例
将PID设置为使用比例测量将使输出在设定值改变时更加平稳。 
另外，它可以消除某些过程中的超调，如真空监视器。
 ********************************************************/

#include <PID_v1.h>

//Define Variables we'll be connecting to
// 定义我们将要连接到的变量
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
// 指定链接和初始调优参数
//P_ON_M specifies that Proportional on Measurement be used
// ponm指定在测量中使用比例
//P_ON_E (Proportional on Error) is the default behavior
// pone(与错误成比例)是默认行为
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, P_ON_M, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  // 初始化我们链接到的变量
  Input = analogRead(0);
  Setpoint = 100;

  //turn the PID on
  // 打开PID
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(0);
  myPID.Compute();
  analogWrite(3, Output);
}
void main()
{
  setup();
  while (true)
  {
    loop();
  }
}