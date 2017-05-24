#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     motorD,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     flag,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     backright,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     frontright,    tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     frontleft,     tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     backleft,      tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    conv,                 tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C4_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
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

  	//Flag

  	if (joy1Btn(7) == 1)
  	{
  		motor[flag] = 100;
  	}
  	else if (joy1Btn(6) == 1)
  	{
  		motor[flag] = -100;
  	}
  	else
  	{
  		motor[flag] = 0;
  	}


   }
}
