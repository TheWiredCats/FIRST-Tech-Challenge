#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S2,     IR,             sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S2,     finbas,         sensorHiTechnicIRSeeker1200)
#pragma config(Motor,  mtr_S1_C1_2,     backright,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     backleft,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     frontleft,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     frontright,        tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
task main()
{
	while(true)
	{
 if (SensorValue[finbas] == 5) // GO FORWARD
 {
	motor[backleft] = -100;
	motor[frontleft] = -100;
	motor[backright] = 100;
  motor[frontright] = 100;
 }
 else if (SensorValue[finbas] < 5) // ROTATE RIGHT
 {
	motor[backleft] = 100;
	motor[frontleft] = 100;
	motor[backright] = 100;
  motor[frontright] = 100;
  }
  else if (SensorValue[finbas] > 5) // ROTATE LEFT
 {
	motor[backleft] = -100;
	motor[frontleft] = -100;
	motor[backright] = -100;
  motor[frontright] = -100;
  }
  else // DOES NOTHING
  {
  motor[backleft] = 0;
	motor[frontleft] = 0;
	motor[backright] = 0;
  motor[frontright] = 0;
  	}
 }
}