#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     BL,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     FL,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     BR,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     FR,            tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    claw,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c";

task main()
{

while(true)
{

getJoystickSettings(joystick);
/*
if(joystick.joy1_y1 > -10 && joystick.joy1_y1 < 10)
{
	motor[BL] = 0;
	motor[FL] = 0;
}
else
{
	motor[BL] = joystick.joy1_y1;
	motor[FL] = joystick.joy1_y1;
}

if(joystick.joy1_y2 > -10 && joystick.joy1_y2 < 10)
{
	motor[BR] = 0;
	motor[FR] = 0;
}
else
{
	motor[BR] = joystick.joy1_y2;
	motor[FR] = joystick.joy1_y2;
}
*/

if(joy1Btn(1) == 1 && joy1Btn(2) == 0 && joy1Btn(3) == 0 && joy1Btn(4) == 0)
{
	motor[BR] = 100;
	motor[FR] = 100;
	motor[BL] = -100;
	motor[FL] = -100;
}
else if(joy1Btn(1) == 0 && joy1Btn(2) == 1 && joy1Btn(3) == 0 && joy1Btn(4) == 0)
{
	motor[BR] = -100;
	motor[FR] = -100;
	motor[BL] = -100;
	motor[FL] = -100;
}
else if(joy1Btn(1) == 0 && joy1Btn(2) == 0 && joy1Btn(3) == 1 && joy1Btn(4) == 0)
{
	motor[BR] = -100;
	motor[FR] = -100;
	motor[BL] = 100;
	motor[FL] = 100;
}
else if(joy1Btn(1) == 0 && joy1Btn(2) == 0 && joy1Btn(3) == 0 && joy1Btn(4) == 1)
{
	motor[BR] = 100;
	motor[FR] = 100;
	motor[BL] = 100;
	motor[FL] = 100;
}
else
{
	motor[BR] = 0;
	motor[FR] = 0;
	motor[BL] = 0;
	motor[FL] = 0;
}








}


}
