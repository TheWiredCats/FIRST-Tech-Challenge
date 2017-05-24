#pragma config(Hubs,  S1, HTMotor,  none,     none,     none)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Hubs,  S3, HTMotor,  none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     Right1,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     Right2,        tmotorfjkfjdiu9uj09Tetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C1_1,     Left1,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     Left2,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_1,     Slide,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_2,     motorI,        tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c"
task main()
{


	while(true)
	{
			getJoystickSettings(joystick);


			if(joystick.joy1_y1 > -10 && joystick.joy1_y1 < 10)
					{
							motor [Left1] = 0;
							motor [Left2] = 0;
					}
			else
			{
					motor [Left1] = joystick.joy1_y1;
					motor [Left2] = joystick.joy1_y1;
			}


			if(joystick.joy2_y1 > -10 && joystick.joy2_y1 < 10)
			{
					motor [Right1] = 0;
					motor [Right2] = 0;
			}
			else
			{
				motor [Right1] = 0;
				motor [Right2] = 0;
			}



}













}