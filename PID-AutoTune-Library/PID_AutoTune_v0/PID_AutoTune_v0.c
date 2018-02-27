
#include "PID_AutoTune_v0.h"
#include "PID_V2.h"
#include <stdlib.h>
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
void PID_ATune_Init(PID_ATune *pid_atunep,double* Input, double* Output)
{
	pid_atunep->input = Input;
	pid_atunep->output = Output;
	pid_atunep->controlType =0 ; //default to PI
	pid_atunep->noiseBand = 0.5;
	pid_atunep->running = false;
	pid_atunep->oStep = 30;
	//TODO:
	PID_ATune_SetLookbackSec(pid_atunep,10);
	pid_atunep->lastTime = millis();
	
}



void PID_ATune_Cancel(PID_ATune *pid_atunep)
{
	pid_atunep->running = false;
} 
 
int PID_ATune_Runtime(PID_ATune *pid_atunep)
{
	pid_atunep->justevaled=false;
	if(pid_atunep->peakCount>9 && pid_atunep->running)
	{
		pid_atunep->running = false;
		//TODO:
		PID_ATune_FinishUp(pid_atunep);
		return 1;
	}
	//TODO:
	unsigned long now = millis();
	
	if((now-pid_atunep->lastTime)<pid_atunep->sampleTime) return false;
	pid_atunep->lastTime =now;
	double refVal = *pid_atunep->input;
	pid_atunep->justevaled=true;
	if(!pid_atunep->running)
	{ //initialize working variables the first time around
		pid_atunep->peakType = 0;
		pid_atunep->peakCount=0;
		pid_atunep->justchanged=false;
		pid_atunep->absMax=refVal;
		pid_atunep->absMin=refVal;
		pid_atunep->setpoint = refVal;
		pid_atunep->running = true;
		pid_atunep->outputStart = *pid_atunep->output;
		*pid_atunep->output = pid_atunep->outputStart+pid_atunep->oStep;
	}
	else
	{
		if(refVal>pid_atunep->absMax)pid_atunep->absMax=refVal;
		if(refVal<pid_atunep->absMin)pid_atunep->absMin=refVal;
	}
	
	//oscillate the output base on the input's relation to the setpoint
	
	if(refVal>pid_atunep->setpoint+pid_atunep->noiseBand) *pid_atunep->output = pid_atunep->outputStart-pid_atunep->oStep;
	else if (refVal<pid_atunep->setpoint-pid_atunep->noiseBand) *pid_atunep->output = pid_atunep->outputStart+pid_atunep->oStep;
	
	
  //bool isMax=true, isMin=true;
  pid_atunep->isMax=true;pid_atunep->isMin=true;
  //id peaks
  for(int i=pid_atunep->nLookBack-1;i>=0;i--)
  {
    double val = pid_atunep->lastInputs[i];
    if(pid_atunep->isMax) pid_atunep->isMax = refVal>val;
    if(pid_atunep->isMin) pid_atunep->isMin = refVal<val;
    pid_atunep->lastInputs[i+1] = pid_atunep->lastInputs[i];
  }
  pid_atunep->lastInputs[0] = refVal;  
  if(pid_atunep->nLookBack<9)
  {  //we don't want to trust the maxes or mins until the inputs array has been filled
	return 0;
	}
  
  if(pid_atunep->isMax)
  {
    if(pid_atunep->peakType==0)pid_atunep->peakType=1;
    if(pid_atunep->peakType==-1)
    {
      pid_atunep->peakType = 1;
      pid_atunep->justchanged=true;
      pid_atunep->peak2 = pid_atunep->peak1;
    }
    pid_atunep->peak1 = now;
    pid_atunep->peaks[pid_atunep->peakCount] = refVal;
   
  }
  else if(pid_atunep->isMin)
  {
    if(pid_atunep->peakType==0)pid_atunep->peakType=-1;
    if(pid_atunep->peakType==1)
    {
      pid_atunep->peakType=-1;
      pid_atunep->peakCount++;
      pid_atunep->justchanged=true;
    }
    
    if(pid_atunep->peakCount<10)pid_atunep->peaks[pid_atunep->peakCount] = refVal;
  }
  
  if(pid_atunep->justchanged && pid_atunep->peakCount>2)
  { //we've transitioned.  check if we can autotune based on the last peaks
    double avgSeparation = (abs(pid_atunep->peaks[pid_atunep->peakCount-1]-pid_atunep->peaks[pid_atunep->peakCount-2])+abs(pid_atunep->peaks[pid_atunep->peakCount-2]-pid_atunep->peaks[pid_atunep->peakCount-3]))/2;
    if( avgSeparation < 0.05*(pid_atunep->absMax-pid_atunep->absMin))
    {
		PID_ATune_FinishUp(pid_atunep);
      pid_atunep->running = false;
	  return 1;
	 
    }
  }
   pid_atunep->justchanged=false;
	return 0;
}

void PID_ATune_FinishUp(PID_ATune *pid_atunep)
{
	  * pid_atunep->output = pid_atunep->outputStart;
      //we can generate tuning parameters!
      pid_atunep->Ku = 4*(2*pid_atunep->oStep)/((pid_atunep->absMax-pid_atunep->absMin)*3.14159);
      pid_atunep->Pu = (double)(pid_atunep->peak1-pid_atunep->peak2) / 1000;
}

double PID_ATune_GetKp(PID_ATune *pid_atunep)
{
	return pid_atunep->controlType==1 ? 0.6 * pid_atunep->Ku : 0.4 * pid_atunep->Ku;
}

double PID_ATune_GetKi(PID_ATune *pid_atunep)
{
	return pid_atunep->controlType==1? 1.2*pid_atunep->Ku / pid_atunep->Pu : 0.48 * pid_atunep->Ku / pid_atunep->Pu;  // Ki = Kc/Ti
}

double PID_ATune_GetKd(PID_ATune *pid_atunep)
{
	return pid_atunep->controlType==1? 0.075 * pid_atunep->Ku * pid_atunep->Pu : 0;  //Kd = Kc * Td
}

void PID_ATune_SetOutputStep(PID_ATune *pid_atunep,double Step)
{
	pid_atunep->oStep = Step;
}

double PID_ATune_GetOutputStep(PID_ATune *pid_atunep)
{
	return pid_atunep->oStep;
}

void PID_ATune_SetControlType(PID_ATune *pid_atunep,int Type) //0=PI, 1=PID
{
	pid_atunep->controlType =Type;
}
int PID_ATune_GetControlType(PID_ATune *pid_atunep)
{
	return pid_atunep->controlType;
}
	
void PID_ATune_SetNoiseBand(PID_ATune *pid_atunep,double Band)
{
	pid_atunep->noiseBand = Band;
}

double PID_ATune_GetNoiseBand(PID_ATune *pid_atunep)
{
	return pid_atunep->noiseBand;
}

void PID_ATune_SetLookbackSec(PID_ATune *pid_atunep,int value)
{
    if (value<1) value = 1;
	
	if(value<25)
	{
		pid_atunep->nLookBack = value * 4;
		pid_atunep->sampleTime = 250;
	}
	else
	{
		pid_atunep->nLookBack = 100;
		pid_atunep->sampleTime = value*10;
	}
}

int PID_ATune_GetLookbackSec(PID_ATune *pid_atunep)
{
	return pid_atunep->nLookBack * pid_atunep->sampleTime / 1000;
}
