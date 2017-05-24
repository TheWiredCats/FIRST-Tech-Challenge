#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Motor,  mtr_S1_C1_1,     BackRight,     tmotorNXT, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     FrontRight,    tmotorNXT, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     armlift,       tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     PullUp,        tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     BackLeft,      tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     FrontLeft,     tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     Flag,          tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     none,          tmotorNXT, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    servoleft,            tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    servoright,           tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    pulluplock,           tServoStandard)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)

#include "JoystickDriver.c"
float wrist_angle;
float pullup_lock;


task main()
{

//Int//
wrist_angle = 255 ;
pullup_lock = 0;

while(true)
{

getJoystickSettings(joystick);

	if	(joystick.joy2_y2 < 10 && joystick.joy2_y2 > -10)

	motor[armlift]=0;

	else

	motor[armlift]= 20;

}
}
