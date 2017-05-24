#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     Leftmotor,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     Rightmotor,     tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     Lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"

task main()
{


while(true)
{
getJoystickSettings(joystick);

//wheel
	if(joystick.joy1_y1 < 10 && joystick.joy1_y1 > -10)

	{
		motor[Leftmotor] = 0;

	}

	else

	{
		motor[Leftmotor] = joystick.joy1_y1;

	}

			if	(joystick.joy1_y2 < 10 && joystick.joy1_y2 > -10)

	{
		motor[Rightmotor] = 0;
	}

	else

	{
		motor[Rightmotor] = joystick.joy1_y2;
	}

	//lift

		if	(joy1Btn(2) == 0)

	{
		motor[Lift] = 0;
	}

	else if (joy1Btn(2) == 1)

	{
		motor[Lift] = 100;
	}




}
}
