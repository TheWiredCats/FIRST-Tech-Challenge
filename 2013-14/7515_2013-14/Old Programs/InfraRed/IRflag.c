#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S2,     finbas,         sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S3,     stop,           sensorTouch)
#pragma config(Motor,  mtr_S1_C1_1,     backleft,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     frontleft,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     flag,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     lifter,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     backright,     tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     frontright,    tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C4_1,    conv,                 tServoContinuousRotation)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#define FORWARD_DIRECTION      1
#define BACKWARDS_DIRECTION   -1
#define STOP                   0

void runAllMotors( int speed );
void stopAllMotors();
void runConveyorBelt( byte direction );
void turnposRightnegleft( int delta );
int basket = 1;
int i = 0;
void initializeRobot()
{
	return;
}
task main()
{
	/*initializeRobot();
	waitForStart();
	 initializeRobot();
	 waitForStart();
   runAllMotors( 50 );
	 wait1Msec(1700);
	 stopAllMotors();
	 wait1Msec(500);
	 turnposRightnegleft(-50);
	 wait1Msec(1200);
	 stopAllMotors();
	 runAllMotors( 50 );
	 wait1Msec(700);
	 stopAllMotors();
	 wait1Msec(500);
for(i = 1; i < 5; i++)
 {
	if (SensorValue[finbas] == 5)
	  {
		   basket = i;
		   i = 97;
		   break;
	  }
	 else if (SensorValue[finbas] != 5)
	    {
	    	if (i == 3 )
	    	 {
	    	   runAllMotors( 50 );
		       wait1Msec(200);
	    	 }
	    	if (i < 4)
	    	{
		      runAllMotors( 50 );
		      wait1Msec(500);
		      stopAllMotors();
		      wait1Msec(1000);
		      if (i >= 4 && basket == 0)
		      {
		    	  basket = 4;
		      }
		     }


      }
  } // end of loop
  if (basket < 6)
	  {
		   turnposRightnegleft( 50 );
		   wait1Msec(1100);
		   runAllMotors( 50 );
	     wait1Msec(400);
	     stopAllMotors();
		   runConveyorBelt(FORWARD_DIRECTION);
		   wait1Msec(1000);
		   runConveyorBelt(BACKWARDS_DIRECTION);
		   wait1Msec(850);
		   runConveyorBelt(STOP);
	     runAllMotors( -50 );
	     wait1Msec(200);
		   turnposRightnegleft( -50 );
		   wait1Msec(1100);
	  }*/
	 if (basket == 1)
  {
	 runAllMotors( 50 );
	 wait1Msec(2700);
  }
  else if (basket == 2)
  {
   runAllMotors( 50 );
	 wait1Msec(2200);
  }
  else if (basket == 3)
  {
   runAllMotors( 50 );
	 wait1Msec(1500);
  }else if (basket == 4)
  {
   runAllMotors( 50 );
	 wait1Msec(1000);
  }
	 turnposRightnegleft( 50 );
	 wait1Msec(850);
	 runAllMotors( 50 );
	 wait1Msec(1200);
   turnposRightnegleft( 50 );
	 wait1Msec(850);
	 runAllMotors( 50 );
	 wait1Msec(1200);
	 stopAllMotors();

}

  /* End of Autonomous
   * Beginning of writing functions*/


  /*First function*/

void runAllMotors( int speed )
{
	/* Make sure that we don't overflow */
	if( speed > 127 )
		speed = 127;
  else if( speed < -127 )
  	speed = -127;

  /* Set motor speed */
	motor[backright] = speed;
  motor[frontright] = speed;
  motor[backleft] = speed;
  motor[frontleft] = speed;
}


  /*Second function*/

void stopAllMotors()
{
	runAllMotors(0); //set all motor power values to 0
}


  /*Third function*/

void runConveyorBelt( byte direction )
{
	if(direction == FORWARD_DIRECTION)
		servo[conv] = (int) 200; // pick any value from 128 to 255
	else if( direction == BACKWARDS_DIRECTION)
		servo[conv] = (int) 100; // pick any value from 0 to 127
  else if( direction == STOP)
    servo[conv] = (int) 127;
  else
  	servo[conv] = 127; //127 stops servo
}


  /*Fourth function*/


void turnposRightnegleft( int delta )
{
	motor[backleft] += delta;
	motor[backright] -= delta;
	motor[frontleft] += delta;
	motor[frontright] -= delta;
}
