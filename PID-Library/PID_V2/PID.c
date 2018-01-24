/**
 * @file    PID_v1.c
 * @brief   pid
 *
 * @addtogroup PID
 * @{
 */
#include "PID.h"
/*===========================================================================*/
/* 模块本地定义.                                                              */
/*===========================================================================*/

/*===========================================================================*/
/* 模块导出变量.                                                              */
/*===========================================================================*/

/*===========================================================================*/
/* 模块本地类型.                                                              */
/*===========================================================================*/

/*===========================================================================*/
/* 模块局部变量.                                                              */
/*===========================================================================*/

/*===========================================================================*/
/* 模块本地功能.                                                              */
/*===========================================================================*/
/* TODO:获取当前时间函数 rtcnt_t chSysGetRealtimeCounterX(void)*/
/* unsigned long (*millis)(void); */
static unsigned long millis(void)
{
    unsigned long ret=0;
#if 0
    systime_t tim = chVTGetSystemTimeX();
    ret = ST2MS(tim);
    chprintf((BaseSequentialStream *)&SD2, "%ld\r\n",ret);
#endif
    return ret;
}
/**
 * @brief 
 * @param pidp 
 * @note  做所有的事情需要发生，以确保从手动模式到自动模式的无扰转换。
 */
void PID_Initialize(PID *pidp)
{
    pidp->outputSum = *pidp->myOutput;
    pidp->lastInput = *pidp->myInput;
    if (pidp->outputSum > pidp->outMax)
        pidp->outputSum = pidp->outMax;
    else if (pidp->outputSum < pidp->outMin)
        pidp->outputSum = pidp->outMin;
}
/**
 * @brief 
 * @param pidp 
 * @param NewSampleTime 
 * @note                 设置执行计算的周期（以毫秒为单位）
 */
void PID_SetSampleTime(PID *pidp, int NewSampleTime)
{
    if (NewSampleTime > 0)
    {
        double ratio = (double)NewSampleTime / (double)pidp->SampleTime;
        pidp->ki *= ratio;
        pidp->kd /= ratio;
        pidp->SampleTime = (unsigned long)NewSampleTime;
    }
}
/**
 * @brief 
 * @param pidp 
 * @param Direction 
 * @note  PID将被连接到一个DIRECT动作过程（+输出导致+输入）或一个REVERSE动作过程（+输出导致-Input。）
 *        我们需要知道哪一个，否则我们可能会增加输出，当我们应该正在减少。 这是从构造函数中调用的。
 */
void PID_SetControllerDirection(PID *pidp, int Direction)
{
    if (pidp->inAuto && Direction != pidp->controllerDirection)
    {
        pidp->kp = (0 - pidp->kp);
        pidp->ki = (0 - pidp->ki);
        pidp->kd = (0 - pidp->kd);
    }
    pidp->controllerDirection = Direction;
}
/*===========================================================================*/
/* 模块导出函数.                                                              */
/*===========================================================================*/
/**
 * @brief               构造函数1
 * @param pidp 
 * @param Input 
 * @param Output 
 * @param Setpoint 
 * @param Kp 
 * @param Ki 
 * @param Kd 
 * @param POn 
 * @param ControllerDirection 
 */
void PID_Init(PID *pidp, PID_Config *pidcfg)
{

    pidp->myOutput = pidcfg->Output;
    pidp->myInput = pidcfg->Input;
    pidp->mySetpoint = pidcfg->Setpoint;
    pidp->inAuto = pidcfg->inAuto;
    /* 默认输出限制对应于arduino pwm限制 */
    PID_SetOutputLimits(pidp, pidcfg->Min, pidcfg->Max);
    //默认的控制器采样时间是0.1秒
    pidp->SampleTime = pidcfg->SampleTime;

    PID_SetControllerDirection(pidp, pidcfg->ControllerDirection);
    PID_SetTunings(pidp, pidcfg->Kp, pidcfg->Ki, pidcfg->Kd, pidcfg->POn);
    /* millis: 函数可获取机器运行的时间长度，单位ms */
    pidp->lastTime = millis() - pidp->SampleTime;
}
#if 0
/**
 * @brief                      构造函数2
 * @param pidp 
 * @param Input 
 * @param Output 
 * @param Setpoint 
 * @param Kp 
 * @param Ki 
 * @param Kd 
 * @param ControllerDirection 
 */
void PID2(PID *pidp, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, int ControllerDirection)
{
    PID_Init(pidp, Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection);
}
#endif

/**
 * @brief         
 * @param pidp 
 * @return true 
 * @return false
 * @note            正如他们所说，这是魔术发生的地方。 每次执行“void
 *                  loop（）”时都应该调用这个函数。 该函数将自行决定是否需要计算一个新的PID输出。
 *                  计算输出时返回true，没有任何操作时返回false。 
 */
bool PID_Compute(PID *pidp)
{
    if (!pidp->inAuto)
        return false;
    /* millis: 获取当前时间 rtcnt_t chSysGetRealtimeCounterX(void)*/
    unsigned long now = millis();
    // unsigned long timeChange = (now - pidp->lastTime);
    // if (timeChange >= pidp->SampleTime)
    /* FIXME:PID时间获取最大为65535,超过会清0 */
    if (1)
    {
        /*计算所有的工作错误变量*/
        double input = *pidp->myInput;
        double error = *pidp->mySetpoint - input;
        double dInput = (input - pidp->lastInput);
        pidp->outputSum += (pidp->ki * error);

        /* 如果指定了P_ON_M，则在测量上添加比例 */
        /* FIXME:pon相关,实际判断都使用了pone */
        if (!pidp->pOnE)
            pidp->outputSum -= pidp->kp * dInput;

        if (pidp->outputSum > pidp->outMax)
            pidp->outputSum = pidp->outMax;
        else if (pidp->outputSum < pidp->outMin)
            pidp->outputSum = pidp->outMin;

        /*如果指定了P_ON_E，则在错误上添加比例*/
        double output;
        /* FIXME:pon相关,实际判断都使用了pone */
        if (pidp->pOnE)
            output = pidp->kp * error;
        else
            output = 0;

        /*计算PID输出的其余部分*/
        output += pidp->outputSum - pidp->kd * dInput;

        if (output > pidp->outMax)
            output = pidp->outMax;
        else if (output < pidp->outMin)
            output = pidp->outMin;
        *pidp->myOutput = output;

        /*记住下次有些变数*/
        pidp->lastInput = input;
        pidp->lastTime = now;
        return true;
    }
    else
        return false;
}
/**
 * @brief 
 * @param pidp 
 * @param Kp 
 * @param Ki 
 * @param Kd 
 * @param POn 
 * @note         该功能可以调整控制器的动态性能。
 *               它是从构造函数自动调用的，但是在正常操作期间也可以调整调音
 * 
 */
void PID_SetTunings(PID *pidp, double Kp, double Ki, double Kd, int POn)
{
    if (Kp < 0 || Ki < 0 || Kd < 0)
        return;
    /* pon可以去掉 */
    pidp->pOn = POn;
    pidp->pOnE = POn == P_ON_E;
#if 0 /* FIXME:可以去掉,这三个变量仅用于前端显示 */
    pidp->dispKp = Kp;
    pidp->dispKi = Ki;
    pidp->dispKd = Kd;
#endif
    double SampleTimeInSec = ((double)pidp->SampleTime) / 1000;
    pidp->kp = Kp;
    pidp->ki = Ki * SampleTimeInSec;
    pidp->kd = Kd / SampleTimeInSec;

    if (pidp->controllerDirection == REVERSE)
    {
        pidp->kp = (0 - pidp->kp);
        pidp->ki = (0 - pidp->ki);
        pidp->kd = (0 - pidp->kd);
    }
}
#if 0
/**
 * @brief 
 * @param pidp 
 * @param Kp 
 * @param Ki 
 * @param Kd 
 * @note        使用最后一次POn设置设置Tunings
 */
void SetTunings2(PID *pidp, double Kp, double Ki, double Kd)
{
    SetTunings1(pidp, Kp, Ki, Kd, pidp->pOn);
}
#endif

/**
 * @brief 
 * @param pidp 
 * @param Min 
 * @param Max 
 * @note             这个函数比SetInputLimits更经常使用。
 而控制器的输入通常会在0-1023范围内（这是默认的），输出会有所不同。
 也许他们会做一个时间窗口，需要0-8000什么的。 或者也许他们会想要从0-125钳。
 谁知道。 无论如何，这一切都可以在这里完成。
 */
void PID_SetOutputLimits(PID *pidp, double Min, double Max)
{
    if (Min >= Max)
        return;
    pidp->outMin = Min;
    pidp->outMax = Max;

    if (pidp->inAuto)
    {
        if (*pidp->myOutput > pidp->outMax)
            *pidp->myOutput = pidp->outMax;
        else if (*pidp->myOutput < pidp->outMin)
            *pidp->myOutput = pidp->outMin;

        if (pidp->outputSum > pidp->outMax)
            pidp->outputSum = pidp->outMax;
        else if (pidp->outputSum < pidp->outMin)
            pidp->outputSum = pidp->outMin;
    }
}
/**
 * @brief  
 * @param pidp 
 * @param Mode 
 * @note        当从手动切换到自动时，允许将控制器模式设置为手动（0）或自动（非零），控制器将自动初始化
 */
void PID_SetMode(PID *pidp, int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !pidp->inAuto)
    { /*我们刚刚从手动到自动*/
        PID_Initialize(pidp);
    }
    pidp->inAuto = newAuto;
}

/**
 * @brief 
 * @param pidp 
 * @return double 
 * @note  只是因为你设置Kp = -1并不意味着它实际上发生了。 这些函数查询PID的内部状态。
 *        他们在这里是为了显示的目的。 例如，这是PID前端使用的功能
 */
#if 0
double GetKp(PID *pidp) { return pidp->dispKp; }
double GetKi(PID *pidp) { return pidp->dispKi; }
double GetKd(PID *pidp) { return pidp->dispKd; }
int GetMode(PID *pidp) { return pidp->inAuto ? AUTOMATIC : MANUAL; }
int GetDirection(PID *pidp) { return pidp->controllerDirection; }
#endif
/** @} */
