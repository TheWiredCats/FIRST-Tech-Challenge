#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     sright,         sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     backleft,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     frontleft,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     flag,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     lifter,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     backright,     tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     frontright,    tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    conv,                 tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C4_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.
float convv;  //these variables controll the ser

void initializeRobot()
{
  return;
}
task main()
{
	convv = 0;
  initializeRobot();

  waitForStart(); // Wait for the beginning of autonomous phase.

  motor[backright] = 100;
  motor[frontright] = 100;
  motor[backleft] = 100;
  motor[frontleft] = 100;
wait1Msec(1000);
convv += 720;        // --- make this spin backwards ---
servo[conv] = (int)convv;
wait1Msec(2000);
motor[backright] = 100;
motor[frontright] = 100;
motor[backleft] = -100;
motor[frontleft] = -100;
wait1Msec(1000);
motor[backright] = 100;
motor[frontright] = 100;
motor[backleft] = 100;
motor[frontleft] = 100;
wait1Msec(2000);
motor[backright] = -100;
motor[frontright] = -100;
motor[backleft] = 100;
motor[frontleft] = 100;
wait1Msec(750);
motor[backright] = 100;
motor[frontright] = 100;
motor[backleft] = 100;
motor[frontleft] = 100;
wait1Msec(500);
}
