#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     sright,         sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     left,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     right,     tmotorTetrix, openLoop)


#include "joystickdriver.c"
task main
{

	while(true)

		getJoystickSettings(joystick);

		//Wheels

		if (joystick.joy1_y1 < 10 && joystick.joy1_y1 > -10) //Right joystick moves back and front right wheels
		{
			motor[right] = 0;

		}
		else
	  {
	  	motor[right] = joystick.joy1_y1;

	  }
  	if (joystick.joy1_y2 < 10 && joystick.joy1_y2 > -10) //Left joystick moves back and front left wheels
  	{
	  	motor[left] = 0;

  	}
  	else
  	{
  		motor[left] = joystick.joy1_y2;
  	}
}
