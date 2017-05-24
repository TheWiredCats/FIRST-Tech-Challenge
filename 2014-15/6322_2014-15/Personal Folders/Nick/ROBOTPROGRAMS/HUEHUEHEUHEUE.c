#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     BackRight,     tmotorNXT, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     FrontRight,    tmotorNXT, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     armlift,       tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     PullUp,        tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     BackLeft,      tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     FrontLeft,     tmotorNXT, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    servoleft,            tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    servoright,           tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    pulluplock,           tServoStandard)
#pragma config(Servo,  srvo_S1_C4_5,    flagturner,           tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
float flag_lift;
float wrist_angle;
float pullup_lock;


task main()
{
//Int//
flag_lift = 0;
wrist_angle = 0;
pullup_lock = 0;

while(true)

{

getJoystickSettings(joystick);


//wheels

	if(joystick.joy1_y1 < 10 && joystick.joy1_y1 > -10)

	{
		motor[FrontLeft] = 0;
		motor[BackLeft] = 0;
	}

	else

	{
		motor[FrontLeft] = joystick.joy1_y1;
		motor[BackLeft] = joystick.joy1_y1;
	}

	if	(joystick.joy1_y2 < 10 && joystick.joy1_y2 > -10)

	{
		motor[FrontRight] = 0;
		motor[BackRight] = 0;
	}

	else

	{
		motor[FrontRight] = joystick.joy1_y2;
		motor[BackRight] = joystick.joy1_y2;
	}


	//Pull Up


	if (	joy2Btn(2) == 1 && joy2Btn(3) == 0)

	{
		motor[PullUp] = -100;
	}

	else if (	joy2Btn(2) == 0 && joy2Btn(3) == 0)

	{
		motor[PullUp] = 0;
	}

	else if ( joy2Btn(2) == 0 && joy2Btn(3) == 1)

	{
		motor[PullUp] = 100;
	}

	else if ( joy2Btn(2) == 0 && joy2Btn(3) == 0)

	{
		motor[PullUp] = 0;
	}


	//Arm Lift
	if (	joy2Btn(6) == 1 && joy2Btn(7) == 0)

	{
		motor[armlift] = -100;
	}

	else if (	joy2Btn(7) == 0 && joy2Btn(6) == 0)

	{
		motor[armlift] = 0;
	}

	else if ( joy2Btn(6) == 1 && joy2Btn(7) == 0)

	{
		motor[armlift] = 100;
	}
	else if ( joy2Btn(6) == 0 && joy2Btn(7) == 0)
	{
		motor[armlift] = 0;
	}


	//Flag Lift controller one


  if	(joy1Btn (1) == 0 && joy1Btn (2) == 1)

	{
  	flag_lift = 255;
  }

  else if (joy1Btn (1) == 0 && joy1Btn (2) == 0)

	{
		flag_lift = 127;
	}

  else if (joy1Btn (1) == 1 && joy1Btn (2) == 0)

	{
  	flag_lift = 0;
  }

  else if (joy1Btn(1) == 0 && joy1Btn(2) == 0)

	{
  	flag_lift = 127;
  }


  //wrist angle


	if  (joystick.joy1_y2 > 10)
	{
		wrist_angle += .7;
	}

	else if	(joystick.joy1_y2 < -10)
  {
  	wrist_angle -=.7;
  }

  //pull-up lock


	if  (joy2Btn(7)== 1 && joy2Btn(6)==0)
	{
		pullup_lock += .9;
	}

	else if	(joy2Btn(7)==0 && joy2Btn(6)==1)
  {
  	pullup_lock -=.9;
  }



	//Setting up range of Servos//



	if(flag_lift < 0)
    		 flag_lift = 0;
      else if(flag_lift > 255)
      	flag_lift = 255;
	if(wrist_angle < 0)
    		 wrist_angle = 0;
      else if(wrist_angle > 255)
      	wrist_angle = 255;
	if(pullup_lock < 0)
					pullup_lock = 0;
			else if(pullup_lock > 255)
				pullup_lock = 0;

  //Int servos//

	servo[servoright] = (int) wrist_angle;
	servo[servoleft] = 255 - (int) wrist_angle;
	servo[flagturner] = (int) flag_lift;
  servo[pulluplock] = (int) pullup_lock;
}
}
