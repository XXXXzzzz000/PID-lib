#include "PID_V2.h"
#include "PID_AutoTune_v0.h"
#include <stdlib.h>

int ATuneModeRemember = 2;
double input = 80, output = 50, setpoint = 180;
double kp = 2, ki = 0.5, kd = 2;

double kpmodel = 1.5, taup = 100, theta[50];
double outputStart = 5;
double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;

bool tuning = false;
unsigned long modelTime, serialTime;
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
PID_ATune pid_aTune;

//set to false to connect to the real world
bool useSimulation = true;

void AutoTuneHelper(bool start);
void changeAutoTune();
void SerialSend();
void SerialReceive();
float myrandom(int X,int Y);
void DoModel();
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
void changeAutoTune()
{
  if (!tuning)
  {
    //Set the output to the desired starting frequency.
    output = aTuneStartValue;
    PID_ATune_SetNoiseBand(&pid_aTune, aTuneNoise);
 PID_ATune_SetOutputStep(&pid_aTune, aTuneStep);
 PID_ATune_SetLookbackSec(&pid_aTune, (int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    PID_ATune_Cancel(&pid_aTune);	
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(bool start)
{
  if (start)
    // ATuneModeRemember = myPID.GetMode();
    ATuneModeRemember = pid.inAuto ? AUTOMATIC : MANUAL;
    
  else
  PID_SetMode(&pid, ATuneModeRemember);
}

void SerialSend()
{
  #if 0 //TODO:待实现
  Serial.print("setpoint: ");
  Serial.print(setpoint);
  Serial.print(" ");
  Serial.print("input: ");
  Serial.print(input);
  Serial.print(" ");
  Serial.print("output: ");
  Serial.print(output);
  Serial.print(" ");
  if (tuning)
  {
    Serial.println("tuning mode");
  }
  else
  {
    Serial.print("kp: ");
    Serial.print(myPID.GetKp());
    Serial.print(" ");
    Serial.print("ki: ");
    Serial.print(myPID.GetKi());
    Serial.print(" ");
    Serial.print("kd: ");
    Serial.print(myPID.GetKd());
    Serial.println();
  }
  #endif
}

void SerialReceive()
{
  #if 0 //TODO:待实现
  if (Serial.available())
  {
    char b = Serial.read();
    Serial.flush();
    if ((b == '1' && !tuning) || (b != '1' && tuning))
      changeAutoTune();
  }
  #endif
}
float myrandom(int X,int Y)
{
  return (float)(rand()%(Y-X+1)+X);
}

void DoModel()
{
  //cycle the dead time
  for (int i = 0; i < 49; i++)
  {
    theta[i] = theta[i + 1];
  }
  //compute the input
  input = (kpmodel / taup) * (theta[0] - outputStart) + input * (1 - 1 / taup) + ((float)myrandom(-10, 10)) / 100;
}

void setup()
{
  PID_Init(&pid, &pid_cfg);
  PID_ATune_Init(&pid_aTune, &input, &output);
  
  if (useSimulation)
  {
    for (int i = 0; i < 50; i++)
    {
      theta[i] = outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid
  PID_SetMode(&pid,AUTOMATIC);

  if (tuning)
  {
    tuning = false;
    changeAutoTune();
    tuning = true;
  }

  serialTime = 0;
  //TODO:串口发送
  // Serial.begin(9600);
}
# if 1 //TODO:调试相关的api待实现
void loop()
{

  unsigned long now = millis();

  if (!useSimulation)
  { //pull the input in from the real world
    //TODO:读取输入
    // input = analogRead(0);
  }

  if (tuning)
  {
    int val = (PID_ATune_Runtime(&pid_aTune));
    if (val != 0)
    {
      tuning = false;
    }
    if (!tuning)
    { //we're done, set the tuning parameters
      kp = PID_ATune_GetKp(&pid_aTune);
      ki = PID_ATune_GetKi(&pid_aTune);
      kd = PID_ATune_GetKd(&pid_aTune);
      PID_SetTunings(&pid,kp, ki, kd,pid.pOn);
      AutoTuneHelper(false);
    }
  }
  else
    PID_Compute(&pid);

  if (useSimulation)
  {
    theta[30] = output;
    if (now >= modelTime)
    {
      modelTime += 100;
      DoModel();
    }
  }
  else
  {
    //TODO:输出
    // analogWrite(0, output);
  }

  //send-receive with processing if it's time
  if (millis() > serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime += 500;
  }
}
#endif

int main ()
{
  setup();
  while (1)
  {
    loop();
  }
return 0;
}
