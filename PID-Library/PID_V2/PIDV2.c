/**
 * @file    PID_v1.c
 * @brief   pid
 *
 * @addtogroup PID
 * @{
 */
#include "PIDV2.h"
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
/**
 * @brief       
 * @param pidp 
 * @param Min 
 * @param Max 
 */
void PID_SetOutputLimits(PID *pidp, float Min, float Max)
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

void PID_SetTunings(PID *pidp, float Kp, float Ki, float Kd, int POn)
{
    if (Kp < 0 || Ki < 0 || Kd < 0)
        return;
    /* pon可以去掉 */
    pidp->pOn = POn;
    pidp->pOnE = POn == P_ON_E;

    float SampleTimeInSec = ((float)pidp->SampleTime) / 1000;
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
/*===========================================================================*/
/* 模块导出函数.                                                              */
/*===========================================================================*/
void PID_Init(PID *pidp, PID_Config *pidcfg)
{

    pidp->Output = pidcfg->Output;
    pidp->Input = pidcfg->Input;
    pidp->Setpoint = pidcfg->Setpoint;
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

bool PID_Compute(PID *pidp)
{
    if (!pidp->inAuto)
        return false;
    /* millis: 获取当前时间 rtcnt_t chSysGetRealtimeCounterX(void)*/
    unsigned long now = millis();
    unsigned long timeChange = (now - pidp->lastTime);
    // if (timeChange >= pidp->SampleTime)
    /* FIXME:PID时间获取最大为65535,超过会清0 */
    if (1)
    {
        /*计算所有的工作错误变量*/
        float input = *pidp->myInput;
        float error = *pidp->mySetpoint - input;
        float dInput = (input - pidp->lastInput);
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
        float output;
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
/** @} */
