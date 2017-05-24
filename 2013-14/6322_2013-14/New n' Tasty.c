#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S1,        ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C2_1,     PlatformLeft,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     PlatformRight, tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_1,     Left,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C3_1,    none,          tServoNone)
#pragma config(Servo,  srvo_S1_C3_2,    none,          tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    none,          tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    ArmJointOne,   tServoStandard)
#pragma config(Servo,  srvo_S1_C3_5,    ArmJointTwo,   tServoStandard)
#pragma config(Servo,  srvo_S1_C3_6,    WristJoint,    tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"

float arm_joint;
float wrist_angle;

task main()

{
wrist_angle = 0;
arm_joint = 127;

while(true)

	{

getJoystickSettings(joystick);


//Motors//


	if(joystick.joy1_y1 < 10 && joystick.joy1_y1 > -10)

		{
			motor[Left] = 0;
		}

	else

		{
			motor[Left] = joystick.joy1_y1;
		}


	if(joystick.joy1_y2 < 10 && joystick.joy1_y2 > -10)

		{
			motor[Right] = 0;
		}

	else

		{
			motor[Right] = joystick.joy1_y2;
		}


//Platform Lift//


	if( joy1Btn(5) == 1 && joy1Btn(6) == 0)

		{
			motor[PlatformLeft] = 100;
			motor[PlatformRight] = 100;
		}

	else if( joy1Btn(5) == 0 && joy1Btn(6) == 1)

		{
			motor[PlatformLeft] = -100;
			motor[PlatformRight] = -100;
		}

	else if( joy1Btn(5) == 0 && joy1Btn(6) == 0)

		{
			motor[PlatformLeft] = 0;
			motor[PlatformRight] = 0;
		}





//Servos, Arm Joint//


	if( joy2Btn(3) == 1 && joy2Btn(4) == 0)

		{
			arm_joint -= .3;
		}

	else if( joy2Btn (3) == 0 && joy2Btn(4) == 1)

		{
			arm_joint += .3;
		}

	else if( joy2Btn (3) == 0 && joy2Btn(4) == 0)

		{
			arm_joint -= .0;
		}


//Servos again, Wrist Joint//


	if(joy2Btn(1) == 1 && joy2Btn(2) == 0)

		{
			wrist_angle += .2;
		}

	else if(joy2Btn(1) == 0 && joy2Btn(2) == 1)

		{
			wrist_angle -= .2;
		}

	else if(joy2Btn(1) == 0 && joy2Btn(2) == 0)

		{
			wrist_angle -= .0;
		}


//Setting Up Range of Servos//


	if(arm_joint < 0)
    		 arm_joint = 0;
      else if(arm_joint > 255)
      	arm_joint = 255;

  if(wrist_angle < 0)
  			wrist_angle = 0;
  		else if(wrist_angle > 255)
  			wrist_angle = 255;


//Int Servo//


  servo[WristJoint] = (int) wrist_angle;
	servo[ArmJointOne] = 255 - (int) arm_joint;
	servo[ArmJointTwo] = (int) arm_joint;
}
}