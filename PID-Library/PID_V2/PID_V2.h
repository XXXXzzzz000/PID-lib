/**
 * @file    PID_v1.h
 * @brief   pid头文件
 *
 * @addtogroup PID
 * @{
 */

#ifndef PID_v1_h
#define PID_v1_h
/*===========================================================================*/
/* 模块预编译设置.                                                            */
/*===========================================================================*/
// #define PID_FRONT 
#define PID_TEST 
/*===========================================================================*/
/* 头文件.                                                                   */
/*===========================================================================*/
#if defined(PID_TEST)
#include <stdbool.h>
#endif
/*===========================================================================*/
/* 模块常量宏定义.                                                            */
/*===========================================================================*/
/* 在下面的一些函数中使用的常量 */
#define AUTOMATIC 1 /* 自动控制输出(启动pid) */
#define MANUAL 0    /* 手动控制输出(暂停pid) */

#define DIRECT 0  /* 输出与反馈为正相关(pid参数不需要反转) */
#define REVERSE 1 /* 输出与反馈为负相关(pid参数需要反转,如输出到冰箱,返回其温度) */

#define P_ON_M 0 /* TODO: 添加注释 */
#define P_ON_E 1

/*===========================================================================*/
/* 派生常量和错误检查.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* 模块数据结构和类型.                                                         */
/*===========================================================================*/
typedef struct
{
        double *Input;
        double *Output;
        double *Setpoint;
        double Kp;
        double Ki;
        double Kd;
        int POn;
        int ControllerDirection;
        unsigned long SampleTime;
        double Min, Max;
        bool inAuto;
} PID_Config;

typedef struct
{
/*我们将保留用户输入格式的调整参数以进行显示*/
#if defined(PID_FRONT)
        double dispKp;
        double dispKi;
        double dispKd;
#endif

        double kp; /*(P)roportional Tuning Parameter 比例调谐参数*/
        double ki; /*(I)ntegral Tuning Parameter 积分调谐参数*/
        double kd; /*(D)erivative Tuning Parameter 微分调谐参数*/

        int controllerDirection; /* 反馈与输出是正相关则为DIRECT 反之则为REVERSE*/
        int pOn;                 /* P_ON_M或 P_ON_E*/

        double *inputp; /* 保存输入输出及设置点 */
        double *outputp;
        double *setpointp;

        unsigned long lastTime;      /* 上一次的输出 */
        double outputSum, lastInput; /* 输出和,上一次的反馈 */
        /*  */
        unsigned long SampleTime; /* 采样频率 */
        double outMin, outMax;    /* 输出钳值 */
        bool inAuto, pOnE;        /* pOnE = POn == P_ON_E */
} PID;
/*===========================================================================*/
/* 模块宏函数.                                                                */
/*===========================================================================*/

/*===========================================================================*/
/* 外部声明.                                                                  */
/*===========================================================================*/

/* 常用功能******************************************************** */

/** 构造函数。 将PID链接到输入，输出和设定点。 初始调整参数也在这里设置。 （指定比例模式的重载） */
void PID_Init(PID *pidp, PID_Config *pidcfg);
/* 将PID设置为手动（0）或自动（非0） */
void PID_SetMode(PID *pidp, int Mode);
/* 执行PID计算。 每次循环都应该调用它。 */
bool PID_Compute(PID *pidp);
/* 将输出钳制到特定范围。 默认情况下为0-255，但用户可能希望根据应用程序进行更改 */
void PID_SetOutputLimits(PID *pidp, double Min, double Max);

/* 可用但不常用的功能 ******************************************************** */

/* 虽然大多数用户会在构造函数中设置一次调谐参数，但是该函数允许用户在运行时为自适应控制更改调谐 */
void PID_SetTunings(PID *pidp, double Kp, double Ki, double Kd, int POn);
/* 设置控制器的方向或“动作”。DIRECT表示错误为正时输出将增加。 REVERSE意味着相反。 它不太可能需要一次性在构造函数中设置好 */
void PID_SetControllerDirection(PID *pidp, int Direction);
/* 设置执行PID计算的频率（以毫秒为单位）。 默认值是100 */
void PID_SetSampleTime(PID *pidp, int NewSampleTime);

/* 显示相关功能 **************************************************************** */
#if defined(PID_FRONT)
double GetKp(PID *pidp);
double GetKi(PID *pidp);
double GetKd(PID *pidp);
int GetMode(PID *pidp);
int GetDirection(PID *pidp);
void Initialize(PID *pidp);
#endif
/*===========================================================================*/
/* 模块内联函数.                                                              */
/*===========================================================================*/

#endif /* PID_v1_h */

/** @} */