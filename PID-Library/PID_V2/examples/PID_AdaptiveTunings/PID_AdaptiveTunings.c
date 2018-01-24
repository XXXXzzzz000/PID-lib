/********************************************************
*PID自适应调节示例
*PID库的好处之一是可以随时更改调整参数。如果我们希望控制器在某些
*时候是激进的，而在其他时候是保守的，这可能会有所帮助。在下面的示
*例中，我们将控制器设置为在靠近设定点时使用保守调谐参数，在距离较
*远时使用更加积极的调谐参数。
 ********************************************************/

#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//定义需要连接的变量
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
// 定义侵略性和保守的调优参数
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

//Specify the links and initial tuning parameters
// 指定链接和初始调优参数
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

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
  // 反馈和设定点的差值
  double gap = abs(Setpoint - Input); //distance away from setpoint
  if (gap < 10)
  { //we're close to setpoint, use conservative tuning parameters
    // 我们接近setpoint，使用保守的调优参数
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    // 我们远离setpoint，使用激进的调优参数
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }

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