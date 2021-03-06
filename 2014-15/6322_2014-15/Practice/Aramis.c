#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     frontleft,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     frontright,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     backleft,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     backright,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     flag,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    sifter,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c"
float pullup_lock;
task main()
{
pullup_lock = 0;
while(true)
{
	getJoystickSettings(joystick);

	if (joystick.joy1_y1 < 10 && joystick .joy1_y1 > -10)
	{
		motor[frontleft] = 0;
		motor[backleft] = 0;
	}
	else
	{
		motor[frontleft] = joystick.joy1_y1;
		motor[backleft] = joystick.joy1_y1;
	}
	if (joystick.joy1_y2 < 10 && joystick.joy1_y2 > -10)
	{
		motor[frontright] = 0;
		motor[backright] = 0;
	}
	else
	{
		motor[frontright] = joystick.joy1_y2;
		motor[backright] = joystick.joy1_y2;
	}

	if (joystick.joy2_y1 < 10 && joystick.joy2_y1 > -10)
	{
		motor[flag] = 0;
	}
	else
	{
		motor[flag] = joystick.joy2_y1;
	}

	if (joy2Btn(1) == 1)
	{
		pullup_lock += 1;
	}
	else if (joy2Btn(2) == 1)
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
