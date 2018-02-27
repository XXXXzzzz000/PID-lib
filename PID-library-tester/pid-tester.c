#include "PID_V2.h"
#include "PID_interfaces.h"
#include <stdlib.h>
//工作变量/初始条件
double setpoint, input, output;
double kp = 1, ki = 2, kd = 0;
const double outputStart = 50;
const double inputStart = 200;
const double setpointStart = 200;

//转换
// PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID pid;
PID_Config pid_cfg = {
    &input,
    &output,
    &setpoint,
    1,
    1,
    1,
    P_ON_M,
    DIRECT,
    100,
    1, 2,
    AUTOMATIC};
//用来模拟连接到pid的进程的参数
#define NTHETA 50
double kpmodel = 1, taup = 50, theta[NTHETA] = {0.0};

bool integrating = false;
int tindex = 0;

//时间变量
unsigned long evalTime = 0, evalInc = 10,
              serialTime = 0, serialInc = 100;
unsigned long now = 0;
static unsigned long millis(void)
{

#if defined(PID_TEST)
  static long ret = 0;
  ret += 100;
#endif

#if !defined(PID_TEST)
  unsigned long ret = 0;
  systime_t tim = chVTGetSystemTimeX();
  ret = ST2MS(tim);
  chprintf((BaseSequentialStream *)&SD2, "%ld\r\n", ret);
#endif

  return ret;
}
float myrandom(int X, int Y)
{
  return (float)(rand() % (Y - X + 1) + X);
}
void SimulateInput()
{
  //使用循环缓冲区创建一个死区时间
  theta[tindex] = output;
  tindex++;
  if (tindex >= NTHETA)
    tindex = 0;

  // 计算输入
  if (integrating)
    input = (kpmodel / taup) * (theta[tindex] - outputStart) + input;
  else
    input = (kpmodel / taup) * (theta[tindex] - outputStart) + (input - inputStart) * (1 - 1 / taup) + inputStart;
  //添加一些噪声

  input += ((float)myrandom(-10, 10)) / 100;
}

void AlterSimulationConditions()
{
  //设定值步进器
  if (now > 49000)
    setpoint = 150;
  else if (now > 44000)
    setpoint = 100;
  else if (now > 38000)
    setpoint = 500;
  else if (now > 36000)
    setpoint = 200;
  else if (now > 32000)
    setpoint = 150;
  else if (now > 20000)
    setpoint = 200;
  else if (now > 11000)
    setpoint = 100;
  else if (now > 8000)
    setpoint = 1000;
  else if (now > 6000)
    setpoint = 200;
  else if (now > 2000)
    setpoint = 150;
  /*else if(now>3000)setpoint=50;*/

  //限制变化
  if (now > 45000)
    PID_SetOutputLimits(&pid, -100, 100);
  else if (now > 39000)
    PID_SetOutputLimits(&pid, 0, 200);
  else if (now > 30000)
    PID_SetOutputLimits(&pid, -255, 255);
  else if (now > 15000)
    PID_SetOutputLimits(&pid, -100, 100);
  else if (now > 9000)
    PID_SetOutputLimits(&pid, 0, 200);

  //随机模式改变
  if (now > 15000)
    PID_SetMode(&pid, AUTOMATIC);
  else if (now > 10900)
    PID_SetMode(&pid, MANUAL);
  else if (now > 8500)
    PID_SetMode(&pid, AUTOMATIC);
  else if (now > 6800)
    PID_SetMode(&pid, MANUAL);
  else if (now > 4500)
    PID_SetMode(&pid, AUTOMATIC);
  else if (now > 4000)
    PID_SetMode(&pid, AUTOMATIC);

  //TODO:改变PID参数
  if (now > 43000)
    PID_SetTunings(&pid, 3, .15, 0.15, pid.pOn);
  else if (now > 39000)
    PID_SetTunings(&pid, 0.5, .1, .05, pid.pOn);
  else if (now > 30000)
    PID_SetTunings(&pid, 0.1, .05, 0, pid.pOn);
  else if (now > 13000)
    PID_SetTunings(&pid, 0.5, 2, 0.15, pid.pOn);
  else if (now > 9000)
    PID_SetTunings(&pid, 2, 1, .05, pid.pOn);

  //model change: switch the nature of the process connected to the pid
  //模型更改：切换连接到pid的进程的性质
  integrating = (now >= 30000);
}

void DoSerial()
{
  static int tmp;
  if (tmp == 0)
  {
    printf("now\tsetpoint\tinput\toutput\r\n");
    tmp = 1;
  }
  //TODO:串口输出
  printf("%ld\t%lf\t%lf\t%lf\r\n", now, setpoint, input, output);
  // PID_Print(now); PID_Print(" ");
  // PID_Print(setpoint); PID_Print(" ");
  // PID_Print(input); PID_Print(" ");
  // PID_Println(output);
}

void setup()
{
  PID_Init(&pid, &pid_cfg);
  //工作变量
  input = inputStart;
  setpoint = setpointStart;
  output = outputStart;
  for (int i = 0; i < NTHETA; i++)
    theta[i] = outputStart;

  //初始化PID
  PID_SetOutputLimits(&pid, -250, 250);
  PID_SetMode(&pid, AUTOMATIC);

  //初始化串口
  // Serial.begin(115200);
  PID_Println("");
  PID_Println("Test Start");
}

void loop()
{

  while (now < evalTime)
  {
    //获取事件函数
    now = millis(); //make sure our evaluations happen at set intervals
    //确保我们的评估按设定的时间间隔进行
  }

  if (now > 60000)
  {
    //串口打印函数
    PID_Println("End Test");
    //在测试结束时阻止执行以获得干净的串行输出
    while (true)
    {
    }
  }
  //根据now的值改变参数
  AlterSimulationConditions();
  // 模拟输入
  SimulateInput();
  // 计算PID输出
  PID_Compute(&pid);

  if (now >= serialTime)
  {
    serialTime += serialInc;
    //通过串口打印输入输出信息
    DoSerial();
  }

  evalTime += evalInc;
}

#if 0
#endif // 0

int main()
{
  setup();

  while (1)
  {
    loop();
  }
  return 0;
}
