#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTMotor,  HTMotor,  none)
#pragma config(Sensor, S3,     IR,             sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S4,     Sonar,          sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     arm_scoop,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     omnitr,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     omnibr,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     raising,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_1,     omnitl,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_2,     omnibl,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_1,     trunk,         tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C3_2,     motorK,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    dtbr,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    rg1,                  tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_1,    dtbl,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    rg3,                  tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    rg2,                  tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    claw,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C1_5,    door,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//READ PURPOSE BELLOW!!!!!!!!!!!!!!!!!!!!!!!

// PURPOSE: OFFICIAL AUTONOMOUS FOR BAE (STARTS OFF ON FLOOR, ONLY KNOCKS DOWN POLE)
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

	servo[claw] = 255;
	servo[dtbr] = 0;
	servo[dtbl] = 0;

	motor[raising] = 100;
	wait1Msec(400);

	runAllMotors(-100);
	wait1Msec(3000);
	rotate(0);

	/*
	motor[raising] = 0;
	while (ref2 > 10)
	{
		servo[claw] = 255;
		wait1Msec(10);
		ref2++;
	}

	runAllMotors(0);
	wait1Msec(500);

	runAllMotors(-100);
	while(SensorValue[IR] != 5 && ref < 400)
	{
		wait1Msec(10);
		ref++;
		servo[claw] = 255;
		servo[dtbr] = 0;
		servo[dtbl] = 0;
	}
	runAllMotors(0);
	wait1Msec(500);
	if(ref <= 220)//pos 3
	{
		rotate(-100);
		wait1Msec(250);

		runAllRight(-100);
		while(SensorValue[Sonar] >= (3*nickfeet))
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
	wait1Msec(500);
	runAllMotors(100);
	wait1Msec(400);

	//knocks down
	runAllMotors(0);
	wait1Msec(1000);
	runAllRight(-100);
	wait1Msec(600);
	runAllMotors(-100);
	wait1Msec(1800);
	rotate(100);
	wait1Msec(600);
	rotate(0);
	runAllMotors(-100);
	wait1Msec(500);
	rotate(-100);
	}
	else if(ref > 220 && ref <= 240)//pos 2
	{
		rotate(-100);
		wait1Msec(750);
		runAllMotors(0);
		wait1Msec(1000);
		runAllMotors(-100);
		wait1Msec(400);
		runAllMotors(0);
		wait1Msec(1000);
		runAllRight(-100);
		while(SensorValue[Sonar] >= (3*nickfeet))
		{
			wait1Msec(10);
		}
		runAllMotors(0);
		runAllRight(100);
		wait1Msec(300);
		rotate(0);
		wait1Msec(100);
		runAllMotors(-100);
		while(SensorValue[Sonar] >= (.5*nickfeet))
		{
			wait1Msec(10);
		}
		rotate(0);
		wait1Msec(300);
		runAllMotors(100);
		wait1Msec(300);
		rotate(0);
		wait1Msec(500);
		runAllMotors(100);
		wait1Msec(400);
		//knocks down
	runAllMotors(0);
	wait1Msec(1000);
	runAllRight(-100);
	wait1Msec(600);
	runAllMotors(-100);
	wait1Msec(1800);
	rotate(-100);
	}
	else if(ref > 240)//pos 1
	{

		rotate(-100);
		wait1Msec(775);
		runAllMotors(-100);
		wait1Msec(1000);
		rotate(0);
		runAllMotors(-100);
		wait1Msec(300);
		runAllMotors(0);
		wait1Msec(100);
		runAllRight(-100);
		while(SensorValue[Sonar] >= (2*nickfeet))
		{
			wait1Msec(10);
		}
		runAllMotors(0);
		wait1Msec(1000);
		runAllMotors(-100);
		while(SensorValue[Sonar] >= (.5*nickfeet))
		{
			wait1Msec(10);
		}
		rotate(0);
		wait1Msec(500);
		runAllMotors(100);
		wait1Msec(400);
		runAllMotors(100);
		wait1Msec(400);
		//knocks down
	runAllMotors(0);
	wait1Msec(1000);
	runAllRight(-100);
	wait1Msec(600);
	runAllMotors(-100);
	wait1Msec(2000);
	rotate(-100);*/
		/*rotate(100);
		wait1Msec(300);
		runAllMotors(-100);
		wait1Msec(1000);
		rotate(-100);
		wait1Msec(1000);*/



	/*<-----------------------------
	if(ref <= 135) // POS 3
	{
	motor[raising] = 100;
	wait1Msec(1000);
	motor[trunk] = 100;
	wait1Msec(800);
	servo[arm_claw] = 155;/*
	rotate(100);
	wait1Msec(100);
	runAllMotors(0);
	wait1Msec(1000);
	runAllRight(-100);
	while(SensorValue(Sonar) > (2.5*nickfeet))
	{
	wait1Msec(10);
	}
	runAllMotors(100);
	wait1Msec(75);
	runAllMotors(0);
	wait1Msec(100);
	runAllRight(-100);
	wait1Msec(50);
	runAllMotors(0);
	wait1Msec(100);
	runAllMotors(100);
	while(SensorValue(Sonar) > (.75*nickfeet))
	{
	wait1Msec(10);
	}
	runAllMotors(0);
	runAllRight(-100);
	wait1Msec(550);
	runAllMotors(100);
	wait1Msec(2000);
	runAllMotors(0);
	}
	else if (ref > 135 && ref <= 180) // POS 2
	{
	rotate(100);
	wait1Msec(100);
	runAllMotors(0);
	wait1Msec(1000);
	runAllRight(100);
	while (SensorValue[Sonar] > (2*nickfeet))
	{
		wait1Msec(10);
	}
	runAllMotors(0);
	wait1Msec(1000);
	runAllMotors(30);
	while (SensorValue[Sonar] > (0.5*nickfeet))
	{
		wait1Msec(10);
	}
	runAllMotors(0);
	wait1Msec(500);
	runAllRight(-100);
	wait1Msec(670);
	runAllMotors(0);
	wait1Msec(500);
	runAllMotors(127);
	wait1Msec(1800);*/
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
