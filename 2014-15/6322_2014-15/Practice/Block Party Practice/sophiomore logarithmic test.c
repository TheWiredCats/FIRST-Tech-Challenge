#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     FL,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     BL,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     FR,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     BR,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     arm,           tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     flag,        tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c";

task main()
{
	while(true)
	{

	if(joystick.joy1_y1 < 10 && joystick.joy1_y1 > -10)
	{
	motor[FL] = 0;
	motor[BL] = 0;
	motor[FR] = 0;
	motor[BR] = 0;
	}
	else
	{
	motor[FL]= pow(joystick.joy1_y1, 2)/100;
	motor[BL]= pow(joystick.joy1_y1, 2)/100;
	motor[FR]= pow(joystick.joy1_y1, 2)/100;
	motor[BR]= pow(joystick.joy1_y1, 2)/100;
	}




	}



}
