/********************************************************
PID RelayOutput示例
和基本的例子一样，这次的示例除了输出到一个数字引脚（我们认为是
控制继电器）。 PID设计为输出模拟值，但继电器只能打开/关闭。 

将它们连接在一起我们使用“时间比例控制”，它本质上是一个非常慢的
PWM版本。 首先我们决定一个窗口大小（5000mS），然后我们设置PID
来调整它在0和窗口大小之间的输出。 最后，我们添加一些逻辑，将
PID输出转换为“继电器接通时间”，其余的窗口为“继电器断开时间”
 ********************************************************/

#include <PID_v1.h>

#define PIN_INPUT 0
#define RELAY_PIN 6

//Define Variables we'll be connecting to
// 定义我们将要连接到的变量
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
// 指定链接和初始调优参数
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

void setup()
{
  windowStartTime = millis();

  //initialize the variables we're linked to
  // 初始化我们链接到的变量
  Setpoint = 100;

  //tell the PID to range between 0 and the full window size
  // 告诉PID到0和全窗口大小之间的范围
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  // 打开PID
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(PIN_INPUT);
  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   * 根据pid输出打开输出开关
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    // 改变中继窗口的时间
    windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime)
    digitalWrite(RELAY_PIN, HIGH);
  else
    digitalWrite(RELAY_PIN, LOW);
}
void main()
{
  setup();
  while (true)
  {
    loop();
  }
}
