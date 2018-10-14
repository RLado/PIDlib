/*
 * Aero-Pendulum + PID code by Ricard Lado
 * Control with serial message in this format: #FunctionCode#SetPoint#Kp#Ki#Kd
 */

#include <PID.h>
#include <Servo.h> //Lib for ESC control 

#define LED_PIN 13 //Lights up when motor can be moving
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 9
#define CYCLE_PIN 4

PID controller;
Servo motor;

int potVal,FC,origin;
unsigned long l_loop;
float MS,Spt,Ang; //Motor speed and setpoint variables

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(CYCLE_PIN, OUTPUT);
  digitalWrite(CYCLE_PIN, HIGH);
  FC=0;
  Spt=1.57;

  //Setup potenciometer
  Serial.println("Setting up Potenciometer");
  origin=analogRead(A0);
  
  //PID controller settings
  Serial.println("Setting up PID Controller");
  controller.SetSampleTime(1.9e-3); //Sample time close to the refresh rate of the ESC
  controller.SetTunings(2.6, 19.6, 0.0867);  //controller.Kp=2.6;  controller.Ki=19.6;  controller.Kd=0.0867;

  //controller.uplim=1;
  controller.uplim=0.3; //Limit thrust: Default value: 0.24
  controller.dwnlim=0;

  //Setup ESC control
  Serial.println("Setting up ESC");
  motor.attach(MOTOR_PIN);
  Serial.println("Sending minimum output");
  motor.writeMicroseconds(MIN_SIGNAL);
  Serial.println("Setup DONE");
}

void serialEvent()  { //Handles serial event, reads function codes and acts as needed
  float kp,ki,kd;
  //Parse serial msg
  FC=Serial.parseInt();
  Spt=Serial.parseFloat();
  kp=Serial.parseFloat();
  ki=Serial.parseFloat();
  kd=Serial.parseFloat();
  controller.SetTunings(kp, ki, kd);
  
  Serial.print(Spt);Serial.print("\t");Serial.print(kp);Serial.print("\t");Serial.print(ki);Serial.print("\t");Serial.print(kd);Serial.println("\t");
  Serial.print("#"); Serial.println(1000+abs(FC)+abs(Spt)+abs(kp)+abs(ki)+abs(kd)); //A better checksum should be implemented
  
  //STOP
  if (FC==0)  {
    //Stop the pendulum and reset the controller
    MS=MIN_SIGNAL;
    motor.writeMicroseconds(MS);
    controller.StopPID();
    controller.Reset();
    digitalWrite(LED_PIN, LOW);
  }
  
  //START
  if (FC==1)  {
    digitalWrite(LED_PIN, HIGH); 
    controller.StartPID();
  }
}

void loop() {
  potVal = analogRead(A0);
  Ang=getRadians();
  controller.clock=micros()/1e6; //in us
  float CtrlSignal=controller.ComputePID(Ang,Spt);
  MS=mapfloat(CtrlSignal,0,1,MIN_SIGNAL,MAX_SIGNAL); //Calculate motor speed, pid out limited to 1

  //Limit Thrust
  //if (MS>ThrustLimit) MS=ThrustLimit;

  //Shutdown if the pendulum flips over. AKA. goes further than 180º
  if (Ang>PI-0.6){
    FC=0;
    MS=MIN_SIGNAL;
    controller.StopPID();
    controller.Reset();
    digitalWrite(LED_PIN, LOW);
  }
  motor.writeMicroseconds(MS); //Move motor

  //THIS IS USED TO READ SCAN TIME FROM THE OSCILOSCOPE
  //Update cycle pin
  if (digitalRead(CYCLE_PIN)==LOW)  {
    digitalWrite(CYCLE_PIN, HIGH);
  }
  else  {
    digitalWrite(CYCLE_PIN, LOW);
  }
  Serial.print(Ang);Serial.print(" | ");Serial.print(Spt);Serial.print(" | ");Serial.print(CtrlSignal);Serial.print(" | ");Serial.print(MS);Serial.print(" | ");Serial.print(controller.clock);Serial.println(" | ");
}

//Complementary function

void calibrateESC() { // This function calibrates the throttle range of the ESC (Read manual)
  Serial.println("This will calibrate the ESC.");
  Serial.println("Now writing maximum output.");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  motor.writeMicroseconds(MAX_SIGNAL);

  // #Wait for input
  while (!Serial.available());
  Serial.read();

  // #Send min output
  Serial.println("Sending minimum output");
  motor.writeMicroseconds(1000);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float getRadians()  { //get radians reading from a potenciometer
  /*
   * int potVal is the current potenciometer value
   * float origin position when the arduino started
   */
  return mapfloat(potVal-origin,0,1023,0,2*PI);
}


