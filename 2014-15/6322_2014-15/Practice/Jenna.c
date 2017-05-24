#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     m1,            tmotorMatrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     m2,            tmotorMatrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     m3,            tmotorMatrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     m4,            tmotorMatrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
#include "joystickdriver.c"
	void initializeRobot()
	{
		return;
	}

task main()
{
	waitForStart ();

	wait1Msec(1000);
	motor[m1] = 100;
	motor[m2] = 100;
	motor[m3] = 100;
	motor[m4] = 100;
	wait1Msec(1000);

	motor[m1] = 100;
	motor[m2] = 100;
	motor[m3] = -100;
	motor[m4] = -100;
	wait1Msec (800);

	motor[m1] = 100;
	motor[m2] = 100;
	motor[m3] = 100;
	motor[m4] = 100;
	wait1Msec(9000);



}
