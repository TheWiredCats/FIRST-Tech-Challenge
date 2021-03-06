#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_2,     flag,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     conv,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     backright,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     frontright,    tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     frontleft,     tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     backleft,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     wheelturner,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motorK,        tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c"
task main()
{
	while(true)
	{
		getJoystickSettings(joystick);

		//Wheels

		if (joystick.joy1_y1 < 10 && joystick.joy1_y1 > -10) //Right joystick moves back and front right wheels
		{
			motor[backright] = 0;
			motor[frontright] = 0;
		}
		else
	  {
	  	motor[backright] = joystick.joy1_y1;
	  	motor[frontright] = joystick.joy1_y1;
	  }
  	if (joystick.joy1_y2 < 10 && joystick.joy1_y2 > -10) //Left joystick moves back and front left wheels
  	{
	  	motor[backleft] = 0;
	  	motor[frontleft] = 0;
  	}
  	else
  	{
  		motor[backleft] = joystick.joy1_y2;
  		motor[frontleft] = joystick.joy1_y2;
  	}

  	//Conveyor Belt

  	if (joystick.joy2_y1 < 10 && joystick.joy2_y1 > -10) //Left joystick moves conveyor belt
  	{
  		motor[conv] = 0;
  	}
  	else
  	{
  		motor[conv] = joystick.joy2_y1;
  	}

  	//Wheel Turner

  	if (joystick.joy2_y2 < 10 && joystick.joy2_y2 > -10)
  	{
  		motor[wheelturner] = 0;
  	}
  	else // <-- this  means, execute statement if joy2Btn(4) == 0
  	{
  		motor[wheelturner] = joystick.joy2_y2;
  	}

}
}
