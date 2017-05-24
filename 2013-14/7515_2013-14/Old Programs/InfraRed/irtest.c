#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S2,     finbas,         sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S3,     stop,           sensorTouch)
#pragma config(Motor,  mtr_S1_C1_1,     backleft,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     frontleft,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     flag,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     lifter,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     backright,     tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     frontright,    tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C4_1,    conv,                 tServoContinuousRotation)
#include "JoystickDriver.c"
task main()
{
	while(true)
	{
 if (SensorValue[finbas] == 5)
 {
	motor[backleft] = 100;
	motor[frontleft] = 100;
	motor[backright] = 100;
  motor[frontright] = 100;
 }
 else if (SensorValue[finbas] < 5)
 {
	motor[backleft] = -100;
	motor[frontleft] = -100;
	motor[backright] = 100;
  motor[frontright] = 100;
  }
  else if (SensorValue[finbas] > 5)
 {
	motor[backleft] = 100;
	motor[frontleft] = 100;
	motor[backright] = -100;
  motor[frontright] = -100;
  }
  else
  {
  motor[backleft] = 0;
	motor[frontleft] = 0;
	motor[backright] = 0;
  motor[frontright] = 0;
  	}
 }
}
