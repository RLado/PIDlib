/*
  PID.cpp - PID library for arduino.
  Created by Ricard Lado, May 7, 2018.
  Released into the public domain.
*/

#include <PID.h>

float PID::ComputePID(float cval,float setpt)	{
	float out=0;
	if(OnOffSw!=1) return out;
	//Check sample time
	if (SampleTime<=clock-lastC){
		lastC=clock;
		
		//Calculate error	
		error=setpt-cval;
		ITerm+=Ki*error;
		
		//Limit ITerm
		if (ITerm<dwnlim) out=dwnlim;
		else if (ITerm>uplim) out=uplim;
		
		//Calculate output of the controller
		out=Kp * error + ITerm - Kd * (cval-lastVal);
		
		lastVal=cval;

		//Limit the controller
		if (out<dwnlim) out=dwnlim;
		else if (out>uplim)	out=uplim;
		
		return out;
	}
}

void PID::SetTunings(float kp,float ki,float kd){
   Kp=kp;
   Ki=ki * SampleTime;
   Kd=kd / SampleTime;
}

void PID::SetSampleTime(float NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio=NewSampleTime/SampleTime;
      Ki*=ratio;
      Kd/=ratio;
      SampleTime=NewSampleTime;
   }
}

void PID::StopPID(){
	OnOffSw=0;
}

void PID::StartPID(){
	OnOffSw=1;
}

void PID::Reset(){
	error=0;
	lastVal=0;
	ITerm=0;
}
