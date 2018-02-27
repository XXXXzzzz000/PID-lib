#ifndef PID_AutoTune_v0
#define PID_AutoTune_v0
#define LIBRARY_VERSION 0.0.1
#include <stdbool.h>
typedef struct
{
	bool isMax, isMin;
	double *input, *output;
	double setpoint;
	double noiseBand;
	int controlType;
	bool running;
	unsigned long peak1, peak2, lastTime;
	int sampleTime;
	int nLookBack;
	int peakType;
	double lastInputs[101];
	double peaks[10];
	int peakCount;
	bool justchanged;
	bool justevaled;
	double absMax, absMin;
	double oStep;
	double outputStart;
	double Ku, Pu;

} PID_ATune;

 void PID_ATune_FinishUp(PID_ATune *pid_atunep);
//commonly used functions **************************************************************************
void PID_ATune_Init(PID_ATune *pid_atunep, double *Input, double *Output); // * Constructor.  links the Autotune to a given PID
int PID_ATune_Runtime(PID_ATune *pid_atunep);															 // * Similar to the PID Compue function, returns non 0 when done
void PID_ATune_Cancel(PID_ATune *pid_atunep);															 // * Stops the AutoTune

void PID_ATune_SetOutputStep(PID_ATune *pid_atunep, double Step); // * how far above and below the starting value will the output step?
double PID_ATune_GetOutputStep(PID_ATune *pid_atunep);

void PID_ATune_SetControlType(PID_ATune *pid_atunep, int Type); // * Determies if the tuning parameters returned will be PI (D=0)
int PID_ATune_GetControlType(PID_ATune *pid_atunep);						//   or PID.  (0=PI, 1=PID)

void PID_ATune_SetLookbackSec(PID_ATune *pid_atunep, int value); // * how far back are we looking to identify peaks
int PID_ATune_GetLookbackSec(PID_ATune *pid_atunep);						 //

void PID_ATune_SetNoiseBand(PID_ATune *pid_atunep, double Band); // * the autotune will ignore signal chatter smaller than this value
double PID_ATune_GetNoiseBand(PID_ATune *pid_atunep);						 //   this should be acurately set

double PID_ATune_GetKp(PID_ATune *pid_atunep); // * once autotune is complete, these functions contain the
double PID_ATune_GetKi(PID_ATune *pid_atunep); //   computed tuning parameters.
double PID_ATune_GetKd(PID_ATune *pid_atunep); //
#endif
