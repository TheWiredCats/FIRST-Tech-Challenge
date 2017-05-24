#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C2_1,     right_wheels,  tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     left_wheels,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     lifting,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     scooper,       tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C1_1,    claw,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    backleft,             tServoStandard)
#pragma config(Servo,  srvo_S1_C1_3,    backright,            tServoStandard)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c"
float daniel;
float chester;
float brad;
float dave;
int linkinpark;
int mike;
int joe;

task main()
{
	daniel = 0;
 	chester = 0;
 	linkinpark = 0;
 	brad = 0;
 	dave = 0;
 	mike = 0;
 	joe = 0;
	while(true)
	{
		getJoystickSettings (joystick);

		//Left Wheels
		if (joystick.joy1_y1 > -10 && joystick.joy1_y1 < 10)
		{
			motor [left_wheels] = 0;
		}
		else
		{
			motor [left_wheels] = ((joystick.joy1_y1)/10);
		}

		//Right Wheels
		if (joystick.joy1_y2 > -10 && joystick.joy1_y2 < 10)//joy1_y1 & joy1_y2 (blue thing 1) joy2_y1 & joy2_y2 (blue thing 2)
		{
			motor [right_wheels] = 0;
		}
		else
		{
			motor [right_wheels] = ((joystick.joy1_y2)/10);
		}

		//Lifting
		if (joystick.joy2_y1 > -10 && joystick.joy2_y1 < 10)
		{
			motor [lifting] = 0;
		}
		else
		{
			motor [lifting] = joystick.joy2_y1;
		}

		//Claw
		if (joy1Btn(2) == 1)
		{
			linkinpark++;
		}
		if (linkinpark%2 != 0 && linkinpark > 0)
		{
			chester += 127;
		}
		else if (linkinpark%2 == 0 && linkinpark > 0)
		{
			chester -= 127;
		}


		//Scooper
	if (joystick.joy2_y2 > -10 && joystick.joy2_y2 < 10)
	{
		motor [scooper] = 0;
	}
	else
	{
		motor [scooper] = joystick.joy2_y2;
	}

	//backleft servo
		if (joy1Btn(3) == 1)
		{
			mike++;
		}
		if (mike%2 != 0 && mike > 0)
		{
			brad += 127;
		}
		else if (mike%2 == 0 && mike > 0)
		{
			brad -= 127;
		}

	//backright servo
			if (joy1Btn(4) == 1)
		{
			joe++;
		}
		if (joe%2 != 0 && joe > 0)
		{
			dave += 127;
		}
		else if (joe%2 == 0 && joe > 0)
		{
			dave -= 127;
		}


	}

	servo [claw] = (int) chester;
	servo [scooper] = (int) daniel;
	servo [backleft] = (int) brad;
	servo [backright] = (int) dave;
}

//Arman mark of approval
//Jarad mark of approval
