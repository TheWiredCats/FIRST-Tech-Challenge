#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Hubs,  S2, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     IR,             sensorHiTechnicIRSeeker1200)
#pragma config(Motor,  mtr_S1_C1_1,     omnibr,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     omnitr,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     raising,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     raising2,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C2_2,     omnitl,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     omnibl,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_1,     trunk,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     intake,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    rg1,                  tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    dtbr,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    rg2,                  tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    arm_scoop,            tServoNone)
#pragma config(Servo,  srvo_S2_C3_1,    rg3,                  tServoStandard)
#pragma config(Servo,  srvo_S2_C3_2,    dtbl,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C3_3,    door,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C3_4,    claw,                 tServoNone)
#pragma config(Servo,  srvo_S2_C3_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//Code for the coolest robot of all time while logged in as root
#include "joystickdriver.c"

bool tank;
float base_servos;
float rg_1;
float rg_2;
float rg_3;
float rs1 = 0, rs2 =0, rs3 = 0;
int one = 0, two = 0, three = 0;
int scoopt = 0;
float doorv = 240, doort = 0;
task main()
{
	base_servos = 5;
	tank = false;
  while (true)
{

	getJoystickSettings(joystick);

	if (base_servos < 50) //Omni-wheel drive
    {
			if (joystick.joy1_x2 < 10 && joystick.joy1_x2 > -10 && joystick.joy1_y1 < 10 && joystick.joy1_y1 > -10 && joystick.joy1_x1 < 10 && joystick.joy1_x1 > -10 && joy1Btn(06) == 0 && joy1Btn(05)== 0 && joystick.joy1_y2 < 10 && joystick.joy1_y2 > -10)
				{
				  motor[omnitl] = 0;
			    motor[omnibl] = 0;
			    motor[omnitr] = 0;
			    motor[omnibr] = 0;
			  }

			// Joy1_y1
			if (joystick.joy1_y1 > 10)
			{
				motor[omnitl] = -joystick.joy1_y1;
			  motor[omnibl] = -joystick.joy1_y1;
			  motor[omnitr] = joystick.joy1_y1;
			  motor[omnibr] = joystick.joy1_y1;
			}
			else if(joystick.joy1_y1 < -10)
			{
				motor[omnitl] = -joystick.joy1_y1;
				motor[omnibl] = -joystick.joy1_y1;
				motor[omnitr] = joystick.joy1_y1;
				motor[omnibr] = joystick.joy1_y1;
			}

			// Joy1_x1
			else if (joystick.joy1_x1 > 10)
			{
				motor[omnitr] = -joystick.joy1_x1;
				motor[omnibr] = joystick.joy1_x1;
				motor[omnitl] = -joystick.joy1_x1;
				motor[omnibl] = joystick.joy1_x1;
			}
			else if (joystick.joy1_x1 < -10)
			{
				motor[omnitr] = -joystick.joy1_x1;
				motor[omnibr] = joystick.joy1_x1;
				motor[omnitl] = -joystick.joy1_x1;
				motor[omnibl] = joystick.joy1_x1;
			}

			//Buttons 5 and 6 False
			if(joy1Btn(05) == 1 && joy1Btn(06) == 0) //Joy1Btn(5) && Joy1Btn(6)
			{
				motor[omnitr] = 127;
				motor[omnibr] = 127;
				motor[omnitl] = 127;
				motor[omnibl] = 127;
			}

			//Buttons 5 and 6 True
			if(joy1Btn(05) == 0 && joy1Btn(06) == 1) //Joy1Btn(5) && Joy1Btn(6)
			{
				motor[omnitr] = -127;
				motor[omnibr] = -127;
				motor[omnitl] = -127;
				motor[omnibl] = -127;
			}

			//Joy1_y2
		  if (joystick.joy1_y2 > 10)
		  {
		  	motor[omnitl] = -joystick.joy1_y2;
		  	motor[omnibr] = joystick.joy1_y2;
		  }
		  else if (joystick.joy1_y2 < -10)
		  {
		  	motor[omnitl] = -joystick.joy1_y2;
		  	motor[omnibr] = joystick.joy1_y2;
		  }

		  //Joy1_x2
			else if (joystick.joy1_x2 > 10)
			{
				motor[omnibl] = -joystick.joy1_x2;
				motor[omnitr] = joystick.joy1_x2; //if you can find this good for you - S3b4ss
			}
			else if (joystick.joy1_x2 < -10)
			{
				motor[omnibl] = -joystick.joy1_x2;
				motor[omnitr] = joystick.joy1_x2;
			}

	} //End if base_servos < 50 close

			else
			{ //Tank-drive when base_servos > 50

	      if (joystick.joy1_y1 < 10 && joystick.joy1_y1 > -10)
				{
					motor[omnibl] = 0;
					motor[omnitl] = 0;
				}
				else
				{
					motor[omnibl] = -joystick.joy1_y1;
					motor[omnitl] = -joystick.joy1_y1;
				}
				if (joystick.joy1_y2 < 10 && joystick.joy1_y2 > -10)
				{
					motor[omnibr] = 0;
					motor[omnitr] = 0;
				}
				else
				{
					motor[omnibr] = joystick.joy1_y2;
					motor[omnitr] = joystick.joy1_y2;
				}

	} //Close else statement when base_servos > 50

			//base_servos
	  	if (joy1Btn(07) == 1 && joy1Btn(08) == 0) //Joy1Btn(7) && Joy1Btn(8)
			{
				base_servos += 5;
			}
			else if (joy1Btn(08) == 1 && joy1Btn(07) == 0) //Joy1Btn(7) && Joy1Btn(8)
			{
				base_servos -= 5;
			}

			//intake

			 if (joy1Btn(4) == 1 && scoopt == 0)
     {
       scoopt = 1;
       motor[intake] = 100;
     }
     else if(joy1Btn(4) == 0 && scoopt == 1)
     {
       scoopt = 2;
     }
     else if(joy1Btn(4) == 1 && scoopt == 2)
     {
      scoopt = 3;
     	motor[intake] = 0;
     }
     else if(joy1Btn(4) == 0 && scoopt == 3)
     {
     	scoopt = 0;
     }


		 //Raising("Linear Slide");
     if(joystick.joy2_y1 < 10 && joystick.joy2_y1 > -10)
     {
       motor[raising2] = 0;
       motor[raising] = 0;
     }
     else
     {
       motor[raising2] = joystick.joy2_y1;
     	 motor[raising] = joystick.joy2_y1;
     }

     //Trunk
     if (joystick.joy2_y2 < 10 && joystick.joy2_y2 > -10)
     {
       motor[trunk] = 0;
     }
     else
     {
       motor[trunk] = -joystick.joy2_y2/3;
     }


     //rg_1 && one

     if (joy1Btn(3) == 1 && one == 0)
     {
    	 one = 1;
     }
     else if(joy1Btn(3) == 0 && one == 1)
     {
       one = 2;
       rs1 = 1;
     }
     else if(joy1Btn(3) == 1 && one == 2)
     {
     	 one = 3;
     }
     else if(joy1Btn(3) == 0 && one == 3)
     {
     	 one = 0;
     	 rs1 = 0;
     }
     if(rs1 == 1)
     {
     	 rg_1 = 127;
     }
     else if(rs1 == 0)
     {
     	 rg_1 = 240;
     }

     //rg_2 && two
     if (joy1Btn(2) == 1 && two == 0)
     {
    	 two = 1;
     }
     else if(joy1Btn(2) == 0 && two == 1)
     {
       two = 2;
       rs2 = 1;
     }
     else if(joy1Btn(2) == 1 && two == 2)
     {
     	 two = 3;
     }
     else if(joy1Btn(2) == 0 && two == 3)
     {
     	 two = 0;
     	 rs2 = 0;
     }
     if(rs2 == 1)
     {
     	 rg_2 = 115;
     }
     else if(rs2 == 0)
     {
     	 rg_2 = 0;
     }

     //rg_3 && three
     if (joy1Btn(1) == 1 && three == 0)
     {
    	 three = 1;
     }
     else if(joy1Btn(1) == 0 && three == 1)
     {
       three = 2;
       rs3 = 1;
     }
     else if(joy1Btn(1) == 1 && three == 2)
     {
     	 three = 3;
     }
     else if(joy1Btn(1) == 0 && three == 3)
     {
     	 three = 0;
     	 rs3 = 0;
     }
     if(rs3 == 1)
     {
     	 rg_3 = 127;
     }
     else if(rs3 == 0)
     {
     	 rg_3 = 30;
     }

     //door
     if(joy2Btn(4) == 1 && doort == 0 )
     {
      doort = 1;
     }
     else if(joy2Btn(4) == 0 && doort == 1)
     {
      doort = 2;
      doorv = 0;
     }
     else if(joy2Btn(4) == 1 && doort == 2)
     {
      doort = 3;
     }
     else if(joy2Btn(4) == 0 && doort == 3)
     {
      doort = 0;
      doorv = 240;
     }

			// Check range base_servos
			if (base_servos < 0)
			base_servos = 0;
			else if(base_servos > 256)
			base_servos = 256;


			// Int declaration
			servo[dtbl] = (int) base_servos;
			servo[dtbr] = (int) base_servos;
			servo[rg1] = (int) rg_1; //MIDDLE SERVO
			servo[rg2] = (int) rg_2; //RIGHT SERVO
			servo[rg3] = (int) rg_3; //LEFT SERVO
			servo[door] = (int) doorv;

		} // Close while (true) loop

} // Close task main()
