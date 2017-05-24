#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_2,      L,             tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_1,      R,             tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,      C,             tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c"
task main()
{
	while(true)
	{
		getJoystickSettings(joystick);

		//For left motor
		if (joystick.joy1_y1 > -10 && joystick.joy1_y1 < 10)
		{
			motor[L] = 0;
		}
 		else
 		{
 		  motor[L] = joystick.joy1_y1;
 		}

 		//For Right Motor
 		if(joystick.joy1_y2 > -10 && joystick.joy1_y2 < 10)
 		{
 		  motor[R] = 0;
		}
 		else
 		{
 	 	 motor[R] = joystick.joy1_y2;
 		}

 		//For Lift
 		if (joy1Btn(1) == 1&& joy1Btn(2) == 0)
		{
			motor[C] = 100;
		}
		else if (joy1Btn(2) == 1&& joy1Btn(1) == 0)
		{
			motor[C] = -100;
		}
		else
		{
			motor[C] = 0;
		}

	}
}