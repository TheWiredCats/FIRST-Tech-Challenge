#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Hubs,  S2, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     RightENC,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motorF,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     Rightlift,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     intake,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C1_2,     LeftENC,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C3_1,     Left,          tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C3_2,     Leftlift,      tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C2_1,    frrg2,                tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    winch,                tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    brrg,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    draw,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    frrg,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C2_1,    blrg,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    flrg,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    flrg2,                tServoStandard)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// PURPOSE: OFFICIAL AUTONOMOUS FOR STATES ROBOT.
#include "joystickdriver.c"

task main()
{
	int drivet = 0; //drive toggle
	int drives = 0;//drive status 0 is drop zone is front 1 is drawbridge is front
	int intk = 0; // conveyor timer
	int intk2 = 0; // zip-tie timer
	int winchv = 0; // winch timer
	int brrgt = 0; //backright rollinggoal timer
	int blrgt = 0; //backleft rollinggoal timer
	int drawv = 0; // draw timer
	int frg = 0; // front right rolling goal timer
	//servo[draw] =  22;
	servo[winch] = 38;

while(true)
{

getJoystickSettings(joystick);


if(joy1Btn(3) == 1 && drivet == 0)
{
drivet = 1;
drives = 1;
}
else if(joy1Btn(3) == 0 && drivet == 1)
{
drivet = 2;
}
else if(joy1Btn(3) == 1 && drivet == 2)
{
drivet = 3;
drives = 0;
}
else if(joy1Btn(3) == 0 && drivet == 3)
{
drivet = 0;
}






if(drives == 0)
{

		if(joystick.joy1_y1 > -10 && joystick.joy1_y1 < 10)
		{
			motor[Left] = 0;
			motor[LeftENC] = 0;

		}
		else
		{
			motor[LeftENC] = joystick.joy1_y1;
			motor[Left] = joystick.joy1_y1;
			}

		if(joystick.joy1_y2 > -10 && joystick.joy1_y2 < 10)
		{
		  motor[Right] = 0;
			motor[RightENC] = 0;

		}
		else
		{
			motor[Right] = joystick.joy1_y2;
			motor[RightENC] = joystick.joy1_y2;

		}
}
else
{

		if(joystick.joy1_y1 > -10 && joystick.joy1_y1 < 10)
		{
			motor[Right] = 0;
			motor[RightENC] = 0;

		}
		else
		{
			motor[RightENC] = -1*(joystick.joy1_y1);
			motor[Right] = -1*(joystick.joy1_y1);
			}

		if(joystick.joy1_y2 > -10 && joystick.joy1_y2 < 10)
		{
		  motor[Left] = 0;
			motor[LeftENC] = 0;

		}
		else
		{
			motor[Left] = -1*(joystick.joy1_y2);
			motor[LeftENC] = -1*(joystick.joy1_y2);

		}

}














if( joy1Btn(2) == 1 && intk == 0)
{
	intk = 1;
	motor[intake] = -100; // conveyor
}
else if(joy1Btn(2) == 0 && intk == 1)
{
	intk = 2;
}
else if(joy1Btn(2) == 1 && intk == 2)
{
	intk = 3;
	motor[intake] = 0; // conveyor
}
else if(joy1Btn(2) == 0 && intk == 3)
{
	intk = 0;
}

/*if(servo[draw] > 21)
{
	motor[intake] = 0; // conveyor
	intk = 0;
	motor[intake2] = 0; // zip-tie
	intk2 = 0;
}
*/



























	if(joy2Btn(2) == 0 && joy2Btn(4) == 1)
	{
		motor[Rightlift] = 100;
		motor[Leftlift] = 100;
	}
	else if(joy2Btn(2) == 1 && joy2Btn(4) == 0)
	{
		motor[Rightlift] = -100;
		motor[Leftlift] = -100;
	}
	else
	{



	if(joystick.joy2_y1 > -10 && joystick.joy2_y1 < 10)
	{
		motor[Leftlift] = 0;
	}
	else
	{
		motor[Leftlift] = joystick.joy2_y1;
	}

	if(joystick.joy2_y2 > -10 && joystick.joy2_y2 < 10)
	{
    motor[Rightlift] = 0;
	}
	else
	{
		motor[Rightlift] = joystick.joy2_y2;
	}




}






















/*
if(joystick.joy2_y1 > -10 && joystick.joy2_y1 < 10 && joystick.joy2_y2 > -10 && joystick.joy2_y2 < 10)
{
		motor[Rightlift] = 0;
		motor[Leftlift] = 0;
}
else if(joystick.joy2_y1 < -10 && joystick.joy2_y1 > 10 && joystick.joy2_y2 > -10 && joystick.joy2_y2 < 10)
{
	motor[Leftlift] = joystick.joy2_y1;
	motor[Rightlift] = 0;

}
else if( joystick.joy2_y1 > -10 && joystick.joy2_y1 < 10 && joystick.joy2_y2 < -10 && joystick.joy2_y2 > 10)
{
	motor[Leftlift] = 0;
	motor[Rightlift] = joystick.joy2_y2;
}
else if(joystick.joy2_y1 < -10 && joystick.joy2_y1 > 10 && joystick.joy2_y2 < -10 && joystick.joy2_y2 > 10)
{
	motor[Leftlift] = joystick.joy2_y1;
	motor[Rightlift] = joystick.joy2_y2;
}
*/













































//26 32 38
if( joy2Btn(3) == 1 && winchv == 0)
{
	winchv = 1;
	servo[winch] = 27;
}
else if(joy2Btn(3) == 0 && winchv == 1)
{
	winchv = 2;
}
else if(joy2Btn(3) == 1 && winchv == 2)
{
	winchv = 3;
	servo[winch] = 26;
}
else if(joy2Btn(3) == 0 && winchv == 3)
{
	winchv = 4;
}
else if(joy2Btn(3) == 1 && winchv == 4)
{
	winchv = 5;
	servo[winch] = 38;
}
else if(joy2Btn(3) == 0 && winchv == 5)
{
	winchv = 0;
}







if( joystick.joy1_TopHat == 2 && brrgt == 0)
{
	brrgt = 1;
	servo[brrg] = 100; // backright rollinggoal
}
else if(joystick.joy1_TopHat != 2 && brrgt == 1)
{
	brrgt = 2;
}
else if(joystick.joy1_TopHat == 2 && brrgt == 2)
{
	brrgt = 3;
	servo[brrg] = 255; // backright rollinggoal
}
else if(joystick.joy1_TopHat != 2 && brrgt == 3)
{
	brrgt = 0;
}

/*
if( joystick.joy1_TopHat == 4 && drawv == 0)
{
	drawv = 1;
	servo[draw] = 22;
}
else if(joystick.joy1_TopHat != 4 && drawv == 1)
{
	drawv = 2;
}
else if(joystick.joy1_TopHat == 4 && drawv == 2)
{
	drawv = 3;
	servo[draw] = 20;
}
else if(joystick.joy1_TopHat != 4 && drawv == 3)
{
	drawv = 4;
}
else if(joystick.joy1_TopHat == 4 && drawv == 4)
{
	drawv = 5;
	servo[draw] = 14;
}
else if(joystick.joy1_TopHat != 4 && drawv == 5)
{
	drawv = 6;
}
else if(joystick.joy1_TopHat == 4 && drawv == 6)
{
	drawv = 7;
	servo[draw] = 20;
}
else if(joystick.joy1_TopHat != 4 && drawv == 7)
{
	drawv = 0;
}

if( joy1Btn(4) == 1 && intk2 == 0)
{
	intk2 = 1;
	motor[intake2] = 100; // zip-tie
}
else if(joy1Btn(4) == 0 && intk2 == 1)
{
	intk2 = 2;
}
else if(joy1Btn(4) == 1 && intk2 == 2)
{
	intk2 = 3;
	motor[intake2] = 0; // zip-tie
}
else if(joy1Btn(4) == 0 && intk2 == 3)
{
	intk2 = 0;
}
*/

if( joystick.joy1_TopHat == 6 && blrgt == 0)
{
	blrgt = 1;
	servo[blrg] = 155; // backleft rollinggoal
}
else if(joystick.joy1_TopHat != 6 && blrgt == 1)
{
	blrgt = 2;
}
else if(joystick.joy1_TopHat == 6 && blrgt == 2)
{
	blrgt = 3;
	servo[blrg] = 0; // backleft rollinggoal
}
else if(joystick.joy1_TopHat != 6 && blrgt == 3)
{
	blrgt = 0;
}



if( joystick.joy1_TopHat == 0 && frg == 0)
{
	frg = 1;
	servo[flrg] = 255; // frontleft rollinggoal
	servo[frrg] = 0;
	servo[frrg2] = 127;
	servo[flrg2] = 0;// frontleft rollinggoal
}
else if(joystick.joy1_TopHat != 0 && frg == 1)
{
	frg = 2;
}
else if(joystick.joy1_TopHat == 0 && frg == 2)
{
	frg = 3;
	servo[flrg] = 0; // frontleft rollinggoal
	servo[frrg] = 255;
	servo[frrg2] = 0;
	servo[flrg2] = 150; // frontleft rollinggoal
}
else if(joystick.joy1_TopHat != 0 && frg == 3)
{
	frg = 0;
}


}

}