#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     Conveyor,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     Lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     Left,          tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    Stopper,                 tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c"
int Stoppert = 0;
task main()
{
while(true)
{
	getJoystickSettings(joystick);

	//Joystick
	//left side
	if(joystick.joy1_y1 < 10 && joystick.joy1_y1 > -10)
		{
			motor[Left] = 0;
		}
	else
		{
			motor[Left] = joystick.joy1_y1;
		}

	//right side
	if(joystick.joy1_y2 < 10 && joystick.joy1_y2 > -10)
		{
			motor[Right] = 0;
		}
	else
		{
			motor[Right] = joystick.joy1_y2;
		}

	//lifiting
	if (joystick.joy2_y1 < 10 && joystick.joy2_y1 > -10)
		{
			motor[Lift] = 0;
		}
	else
		{
			motor[Lift] = joystick.joy2_y1;
		}
	//conveyor
		if(joystick.joy2_y2 < 10 && joystick.joy2_y2 > -10)
		{
			motor[Conveyor] = 0;
		}
		else
		{
			motor[Conveyor] = joystick.joy2_y2;
		}
	//claw
		if(joy2Btn(2) == 1 && Stoppert == 0)
		{
			servo[Stopper] = 127;
			Stoppert = 1;
		}
		else if(joy2Btn(2) == 0 && Stoppert == 1)
		{
			Stoppert = 2;
		}
		else if(joy2Btn(2) == 1 && Stoppert == 2)
		{
			servo[Stopper] = 0;
			Stoppert = 3;
		}
		else if(joy2Btn(2) == 0 && Stoppert == 3)
		{
			Stoppert = 0;
		}

}
}
