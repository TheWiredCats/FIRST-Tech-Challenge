#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     sright,         sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     backleft,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     frontleft,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     flag,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     lifter,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     backright,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     frontright,    tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C4_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    sifter,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c"
float pullup_lock;

task main()
{
	int x = 1;
	pullup_lock = 0;
	while(true)
	{
		getJoystickSettings(joystick);

		//	Wheels
    if (joy1Btn(8) == 1)
    {
      x = 4;     //we divided the joystick value by x to controll the speed when rasing the flag
    }
    else
    {
      x = 1;
    }
		if (joystick.joy1_y1 < 10 && joystick.joy1_y1 > -10) //Right joystick moves back and front right wheels
		{
			motor[backright] = 0;
			motor[frontright] = 0;
		}
		else
	  {
	  	motor[backright] = (joystick.joy1_y1)/x;
	  	motor[frontright] = (joystick.joy1_y1)/x;
	  }
  	if (joystick.joy1_y2 < 10 && joystick.joy1_y2 > -10) //Left joystick moves back and front left wheels
  	{
	  	motor[backleft] = 0;
	  	motor[frontleft] = 0;
  	}
  	else
  	{
  		motor[backleft] = (joystick.joy1_y2)/x;
  		motor[frontleft] = (joystick.joy1_y2)/x;
  	}

  	//Flag

  	if (joy1Btn(6) == 1)
  	{
  		motor[flag] = 100;
  	}
  	else if (joy1Btn(4) == 1)
  	{
  		motor[flag] = -100;
  	}
  	else
  	{
  		motor[flag] = 0;
  	}

  	//Lifting Mechanism

  	if (joy1Btn(7) == 1)
  	{
  		motor[lifter] = 100;
  	}
  	else if (joy1Btn(5) == 1)
  	{
  	  motor[lifter] = -100;
  	}
  	else
  	{
  		motor[lifter] = 0;
  	}

  //pull-up lock


	if  (joy1Btn(1) == 1)
	{
		pullup_lock += 1;
	}

	else if	(joy1Btn(2) == 1)
  {
  	pullup_lock -= 1;
  }

  	//Check Range
  	if (pullup_lock < 0)
    pullup_lock = 0;
    else if(pullup_lock > 255)
    pullup_lock = 255;

    //Int servos
   	servo[sifter] = (int) pullup_lock;

   }
}
