#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S2, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S3,     Gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C2_1,     RightENC,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     Right,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     intake2,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     Rightlift,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     intake,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C1_2,     LeftENC,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C3_1,     Left,          tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C3_2,     Leftlift,      tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C1_1,    frrg2,                tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    winch,                tServoStandard)
#pragma config(Servo,  srvo_S1_C1_3,    brrg,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C1_4,    draw,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    frrg,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C2_1,    blrg,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    flrg,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    flrg2,                tServoStandard)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// PURPOSE: OFFICIAL AUTONOMOUS FOR STATES ROBOT.
#include "joystickdriver.c"


int initialTurnReading;
float degreesSoFar;
int currentGyroReading;
int ref =0;


void rotate(int degreesToTurn);
void forward(int speed);
void lift(int speed , int time);

task main()
{
	servo[draw] = 22;
	servo[flrg] = 255;
	servo[frrg] = 0;
	servo[frrg2] = 255;
	servo[flrg2] = 0;
	servo[winch] = 38;
waitForStart();

forward(42);
wait1Msec(2000);
forward(0);
wait1Msec(500);
forward(42);
wait1Msec(700);
forward(0);
while( ref < 30)
{
	wait1Msec(10);
	servo[flrg] = 0;
	servo[frrg] = 255;
	servo[frrg2] = 0;
	servo[flrg2] = 255;
	ref++;
}
wait1Msec(300);
forward(-100);
wait1Msec(150);
forward(0);
wait1Msec(200);

motor[Leftlift] = 100;
motor[Rightlift] = 100;
wait1Msec(1900);
motor[Leftlift] = 0;
motor[Rightlift] = 0;

servo[winch] = 29;
wait1Msec(9000);
servo[winch] = 38;
wait1Msec(3000);
lift(-100, 1500);
wait1Msec(300);
rotate(120);
wait1Msec(500);
forward(100);
wait1Msec(3000);
forward(0);

}


void lift(int speed , int time)
{
motor[Leftlift] = speed;
motor[Rightlift] = speed;
wait1Msec(time);
motor[Leftlift] = 0;
motor[Rightlift] = 0;


}




void forward(int speed)
{
	motor[LeftENC] = speed;
	motor[Left] = speed;
	motor[Right] = speed;
	motor[RightENC] = speed;

}


void rotate(int degreesToTurn)
{

	degreesSoFar = 0;
	initialTurnReading = SensorValue(Gyro);

	// start turning
	motor[LeftENC] = -1*(50*sgn(degreesToTurn));
	motor[Left] = -1*(50*sgn(degreesToTurn));
	motor[Right] = (50*sgn(degreesToTurn));
	motor[RightENC] = (50*sgn(degreesToTurn));




	// check if we have turned enough
	while(abs(degreesSoFar) < degreesToTurn)
	{
		// update degreesSoFar
		wait1Msec(10);
		currentGyroReading = SensorValue(Gyro) - initialTurnReading;
		degreesSoFar = degreesSoFar + currentGyroReading*.01;
	}

	// stop turning
	motor[LeftENC] = 0;
	motor[Left] = 0;
	motor[Right] = 0;
	motor[RightENC] = 0;


}
//hai
