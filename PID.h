/*
  PID.h - PID library for arduino.
  Created by Ricard Lado, May 7, 2018.
  Released into the public domain.
*/
#ifndef PID_h
#define PID_h

class PID
{
  public:
    //Variables
	float SampleTime=1e-3; //Default settings expect seconds. But as long as the time units used are consistent any time unit is valid.
	float clock=0; //Clock variable
	
	//PID output limits
	float uplim=2000;
	float dwnlim=0;
	  
    //Functions
    float ComputePID(float cval, float setpt);
    void SetTunings(float kp, float ki, float kd);
    void SetSampleTime(float NewSampleTime);
    void StopPID();
    void StartPID();
	void Reset(); //Resets PID private variables
  private:
  	//Variables
  	float Kp=1;
	float Ki=0;
	float Kd=0;
	
  	int OnOffSw=0; //PID is OFF by default
    float error=0;
	float ITerm=0;
	float lastC=0; //Time last cycle
	float lastVal=0; //Last current value
};

#endif
