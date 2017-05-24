#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Hubs,  S2, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S3,     IR,             sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S4,     Sonar,          sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     omnibr,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     omnitr,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     raising,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     raising2,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C1_1,     omnitl,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     omnibl,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_1,     trunk2,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C2_2,     trunk,         tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    rg1,                  tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    dtbr,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    rg2,                  tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    arm_scoop,            BadType)
#pragma config(Servo,  srvo_S2_C3_1,    rg3,                  tServoStandard)
#pragma config(Servo,  srvo_S2_C3_2,    dtbl,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C3_3,    door,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C3_4,    claw,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C3_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// PURPOSE: OFFICIAL AUTONOMOUS FOR BAE :)
#include "joystickdriver.c"
void rotate ( int speed );
void runAllMotors ( int speed );
void runAllRight ( int speed );
void rise ( int speed );
float nickfeet = 30;
int ref = 0;
int ref2 = 0;


task main()
{
	waitForStart();
	servo[rg2] = 127;
	servo[door] = 255;
	servo[claw] = 255;
	servo[dtbr] = 0;
	servo[dtbl] = 0;

	motor[raising] = 100;
	wait1Msec(400);
	motor[raising] = 0;

	runAllMotors(0);
	wait1Msec(500);

	runAllMotors(-100);
	while(SensorValue[IR] != 5 && ref < 276)
	{
		wait1Msec(10);
		servo[claw] = 255;
		servo[dtbr] = 0;
		servo[dtbl] = 0;
		servo[door] = 255;
		ref++;
	}
	runAllMotors(0);
	wait1Msec(500);
	if(ref <= 250)//pos 3
	{
		rotate(-100);
		wait1Msec(220);
		runAllRight(-100);
		while(SensorValue[Sonar] >= (2*nickfeet))
	{
		wait1Msec(10);
	}
	runAllMotors(0);
	wait1Msec(1000);
	runAllMotors(-50);
	while(SensorValue[Sonar] >= (0.5*nickfeet))
	{
		wait1Msec(10);
	}
	runAllMotors(0);
	wait1Msec(100);
	runAllMotors(100);
	wait1Msec(300);
	rotate(0);



		//score start
			motor[raising] = 100;
	motor[raising2] = 100;
	ref2 = 0;
	while(ref2 < 320)
	{
		servo[claw] = 255;
		wait1Msec(10);
		ref2++;
	}
	ref2 = 0;
	motor[raising] = 0;
	motor[raising2] = 0;
	motor[trunk] = 50;
	motor[trunk2] = 50;
	while(ref2 < 80)
	{
		servo[claw] = 255;
		wait1Msec(10);
		ref2++;
	}
	ref2 = 0;
	motor[trunk] = 0;
	motor[trunk2] = 0;
	wait1Msec(1000);
	servo[door] = 0;
	wait1Msec(1000);
	servo[door] = 255;
	/*
	motor[trunk] = -50;
	motor[trunk2] = -50;
	while(ref2 < 85)
	{
		servo[claw] = 255;
		wait1Msec(10);
		ref2++;
	}
	ref2 = 0;
	motor[trunk] = 0;
	motor[trunk2] = 0;
	*/
	wait1Msec(500);
	runAllMotors(100);
	wait1Msec(300);
	rotate(0);
	wait1Msec(500);
	motor[raising] = -100;
	motor[raising2] = -100;
	while(ref2 < 100)
	{
		servo[claw] = 255;
		wait1Msec(10);
		ref2++;
	}
	ref2 = 0;
	motor[raising] = 0;
	motor[raising2] = 0;
	//score end

	runAllMotors(0);
		wait1Msec(1000);
		runAllRight(-100);
		wait1Msec(650);
		runAllMotors(-100);
		wait1Msec(2000);



	}
	else if(ref > 250 && ref < 276)//pos 2
	{

		rotate(-100);
		wait1Msec(650);
		runAllMotors(0);
		wait1Msec(1000);
		runAllMotors(-100);
		wait1Msec(600);
		runAllMotors(0);
		wait1Msec(1000);
		runAllRight(-100);
		while(SensorValue[Sonar] >= (2*nickfeet))
		{
			wait1Msec(10);
		}
		runAllMotors(0);
		wait1Msec(300);
		runAllRight(-100);
		wait1Msec(150);
		rotate(0);
		wait1Msec(300);
		runAllMotors(-100);
		while(SensorValue[Sonar] >= (.5*nickfeet))
		{
			wait1Msec(10);
		}
		rotate(0);
		wait1Msec(300);
		runAllMotors(100);
		wait1Msec(200);
		rotate(0);

		//score start
			motor[raising] = 100;
	motor[raising2] = 100;
	ref2 = 0;
	while(ref2 < 320)
	{
		servo[claw] = 255;
		wait1Msec(10);
		ref2++;
	}
	ref2 = 0;
	motor[raising] = 0;
	motor[raising2] = 0;
	motor[trunk] = 50;
	motor[trunk2] = 50;
	while(ref2 < 80)
	{
		servo[claw] = 255;
		wait1Msec(10);
		ref2++;
	}//trolololololololololololololololololololololololololololololololololololololololololololololololololololololololololololololololololololololol
	ref2 = 0;
	motor[trunk] = 0;
	motor[trunk2] = 0;
	wait1Msec(1000);
	servo[door] = 0;
	wait1Msec(1000);
	servo[door] = 255;
	/*
	motor[trunk] = -50;
	motor[trunk2] = -50;
	while(ref2 < 85)
	{
		servo[claw] = 255;
		wait1Msec(10);
		ref2++;
	}
	ref2 = 0;
	motor[trunk] = 0;
	motor[trunk2] = 0;
	*/
	wait1Msec(500);
	runAllMotors(100);
	wait1Msec(300);
	rotate(0);
	wait1Msec(500);
	motor[raising] = -100;
	motor[raising2] = -100;
	while(ref2 < 100)
	{
		servo[claw] = 255;
		wait1Msec(10);
		ref2++;
	}
	ref2 = 0;
	motor[raising] = 0;
	motor[raising2] = 0;
	//score end

		runAllMotors(0);
		wait1Msec(1000);
		runAllRight(-100);
		wait1Msec(650);
		runAllMotors(-100);
		wait1Msec(2000);

		}
		else // pos 1
		{

		runAllMotors(0);
		wait1Msec(1000);
		rotate(-100);
		wait1Msec(1000);
		rotate(0);
		wait1Msec(500);
		runAllMotors(-100);
		wait1Msec(2500);
		}
	}






void rotate ( int speed )
{
	motor[omnitl] = speed;
	motor[omnitr] = speed;
	motor[omnibl] = speed;
	motor[omnibr] = speed;
}
void runAllMotors( int speed )
{
	if (speed > 127)
		speed = 127;
	else if( speed < -127)
		speed = -127;

		motor[omnitr] = speed;
		motor[omnibr] = speed;
		motor[omnibl] = -speed;
		motor[omnitl] = -speed;
}
void runAllRight( int speed ) // Horizontal DIR
{
	motor[omnitr] = -speed;
	motor[omnibr] = speed;
	motor[omnitl] = -speed;
	motor[omnibl] = speed;
}
void rise( int speed )
{
	motor[raising] = speed;
	wait1Msec(400);
	motor[raising] = 0;
	while (ref2 > 40)
	{
		servo[claw] = 255;
		servo[door] = 255;
		wait1Msec(10);
		ref2++;
	}

	runAllMotors(0);
	wait1Msec(500);
}
