#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     FR,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     BR,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     clawlift,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     pullup,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     FL,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     BL,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     flag,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     motorK,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    rightclaw,            tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    lock,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C4_5,    leftclaw,             tServoStandard)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c";


task main()
{
	float rightclawv = 255;
	float leftclawv  = 255;
	float lockv       = 255;

	while(true)
	{
		if(joystick.joy1_y1 > -10 && joystick.joy1_y1 < 10)
		{
			motor[FL] = 0;
			motor[BL] = 0;
		}
		else
		{
			motor[FL] = joystick.joy1_y1;
			motor[BL] = joystick.joy1_y1;
		}

		if(joystick.joy1_y2 > -10 && joystick.joy1_y2 < 10)
		{
			motor[FR] = 0;
			motor[BR] = 0;
		}
		else
		{
			motor[FR] = joystick.joy1_y2;
			motor[BR] = joystick.joy1_y2;
		}

		if(joy1Btn(3) == 1 && joy1Btn(2) == 0)
    {
    	motor[clawlift] = 100;
    }
    else if(joy1Btn(2) == 1 && joy1Btn(3) == 0)
    {
    	motor[clawlift] = -100;
    }
    else
    {
    motor[clawlift] = 0;
    }

    if(joy1Btn(1) == 1 && joy1Btn(4) == 0)
    {
    	motor[pullup] = 100;
    }
    else if(joy1Btn(4) == 1 && joy1Btn(1) == 0)
    {
    	motor[pullup] = -100;
    }
    else
    {
    	motor[pullup] = 0;
    }

    if(joy1Btn(2) == 1 && joy1Btn(3) == 0)
    {
    	motor[flag] = 100;
    }
    else if(joy1Btn(3) == 1 && joy1Btn(2) == 0)
    {
    	motor[flag] = -100;
    }
    else
    {
    	motor[flag] = 0;
    }

    if(joy2Btn(1) == 1 && joy2Btn(4) == 0)
    {
    	rightclawv -= .5;
    }
    else if(joy2Btn(4) == 1 && joy2Btn(1) == 0)
    {
    	rightclawv += .5;
    }

    if(joy2Btn(2) == 1 && joy2Btn(3) == 0)
    {
    	leftclawv -= .5;
    }
    else if(joy2Btn(3) == 1 && joy2Btn(2) == 0)
    {
    	leftclawv += .5;
    }

    if(joy1Btn(5) == 1 && joy1Btn(7) == 0)
    {
    	lockv -= .5;
    }
    else if(joy1Btn(7) == 1 && joy1Btn(5) == 0)
    {
    	lockv += .5;
    }

  servo[rightclaw] = (int)rightclawv;
	servo[leftclaw] = (int)leftclawv;
	servo[lock] = (int)lockv;


	}



}
