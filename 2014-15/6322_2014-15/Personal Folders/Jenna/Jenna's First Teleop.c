#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C2_1,     RightWheel,    tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     LeftWheel,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     Lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C1_1,    Scoop,                tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    Scoopturn,            tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C1_3,    Hook,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c"

task main()
{
float scoopv = 0;
int hookt = 0;
int scoopt = 0;
		while(true)
  	{
				getJoystickSettings(joystick);




  	if(joystick.joy1_y1 > -10 && joystick.joy1_y1 < 10) //Left side wheels with left joystick controller 1
  	{
  			motor [LeftWheel] = 0;
    }
 		else
 		{
 				motor [LeftWheel] = joystick.joy1_y1;
 	  }



 	 if (joystick.joy1_y2 > -10 && joystick.joy1_y2 < 10) //right side wheels with right joystick controller 1
 		{
 				motor [RightWheel] = 0;
 	  }
 	 else
 	  {
 	   		motor [RightWheel] = joystick.joy1_y2;
 	  }



 	   if (joystick.joy2_y1 > -10 && joystick.joy1_y2 < 10) //Linear slide with left joystick controller 2
 	   {
 	 		 motor [Lift] = 0;
 	   }
	  else
	 {
			motor [Lift] = joystick.joy2_y1;
   }



 	  if (joy2Btn(4) == 1 && scoopt == 0) //Scoop turn with button Y controller 1
 	  {
 				scoopt = 1;
 				servo [Scoopturn] = 0;
 	  }
 	  else if (joy2Btn(4) == 0 && scoopt == 1)
 	  {
 	  		scoopt = 2;
 	  }
 	  else if (joy2Btn(4) == 1 && scoopt == 2)
 	  {
 	  	  scoopt = 3;
 	  	  servo [Scoopturn] = 127;
 		}
 	  else if (joy2Btn(4) == 0 && scoopt == 3)
 	  {
					scoopt = 0;
 	  }


 	  if (joy1Btn(2) == 1 && hookt == 0) // Hook with button A controller 1
 	  {
 	  		hookt = 1;
 	  		servo [Hook] = 0;
 		}
 		else if (joy1Btn(2) == 0 && hookt == 1)
 		{
 				hookt = 2;
 		}
 		else if(joy1Btn(2) == 1 && hookt == 2)
 		{
 			hookt = 3;
 			servo [Hook] = 127;
 		}
 		else if(joy1Btn(2) == 0 && hookt == 3)
 		{
 			hookt = 0;
 		}


 	  if (joystick.joy2_y2 < -10 ) // Move scoop with joystick 2 controller 2
 	  {
 	  		scoopv -= 5;
 		}
 	  else if(joystick.joy2_y2 > 10)
 	  {
 	  		scoopv += 5;
 		}

 		if(scoopv > 255)
 			scoopv = 255;
 		else if(scoopv < 0)
 			scoopv = 0;

 		servo[Scoop] = (int)scoopv;


 		}

 	}
