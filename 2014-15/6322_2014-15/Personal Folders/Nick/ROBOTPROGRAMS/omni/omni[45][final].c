#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTMotor,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     IR,             sensorHiTechnicIRSeeker1200)
#pragma config(Motor,  mtr_S1_C1_1,     arm_scoop,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     omnitr,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     omnibr,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     raising,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_1,     omnitl,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_2,     omnibl,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_1,     trunk,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_2,     motorK,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    dtbr,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    rg1,                  tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_1,    dtbl,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    rg3,                  tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    rg2,                  tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//Code for the coolest robot of all time while logged in as root
#include "joystickdriver.c"

bool tank;
float base_servos;
float rg_1;
float rg_2;
float rg_3;
int one = 0, two = 0, three = 255;
task main()
{
	base_servos = 5;
	tank = false;
  while (true)
{

	getJoystickSettings(joystick);

	if (base_servos < 50) // Omni-wheel Drive
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

			// Buttons 5 and 6 False
			if(joy1Btn(05) == 1 && joy1Btn(06) == 0) //Joy1Btn(5) && Joy1Btn(6)
			{
				motor[omnitr] = 127;
				motor[omnibr] = 127;
				motor[omnitl] = 127;
				motor[omnibl] = 127;
			}

			// Buttons 5 and 6 True
			if(joy1Btn(05) == 0 && joy1Btn(06) == 1) //Joy1Btn(5) && Joy1Btn(6)
			{
				motor[omnitr] = -127;
				motor[omnibr] = -127;
				motor[omnitl] = -127;
				motor[omnibl] = -127;
			}

			// Joy1_y2
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

		  // Joy1_x2
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

	} // if base_servos < 50 close

			else
			{ // Tank drive when base_servos > 50

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

	} // close else statement when base_servos > 50

			// Button 7 True and Button 8 False
	  	if (joy1Btn(07) == 1 && joy1Btn(08) == 0) //Joy1Btn(7) && Joy1Btn(8)
			{
				base_servos += 5;
			}
			// Button 8 True and Button 7 False
			else if (joy1Btn(08) == 1 && joy1Btn(07) == 0) //Joy1Btn(7) && Joy1Btn(8)
			{
				base_servos -= 5;
			}
			// Second controller
     if(joystick.joy2_y1 < 10 && joystick.joy2_y1 > -10)
     {
       motor[raising] = 0;
     }
     else
     {
       motor[raising] = joystick.joy2_y1;
     }
     if (joystick.joy2_y2 < 10 && joystick.joy2_y2 > -10)
     {
       motor[trunk] = 0;
     }
     else
     {
       motor[trunk] = joystick.joy2_y2/10;
     }
     if (joy1Btn(9) == 1)
     {
       motor[arm_scoop] = 100;
     }
     else
     {
      motor[arm_scoop] = 0;
     }
     if (joy1Btn(4) == 1) //Joy1Btn(1)
     {
       rg_1 = 150;
    	 one++;
     }
     if (one % 2 == 0)
     {
     		rg_1 = 255;
     }
     else
     {
     		rg_1 = 150;
     }
     if (joy1Btn(2) == 1) //Joy1Btn(2)
     {
       rg_2 = 150;
       two++;
     }
     if (two % 2 == 0)
     {
       rg_2 = 255;
     }
     else
     {
       rg_2 = 150;
     }
     if (joy1Btn(1) == 1) //Joy1Btn(3)
     {
       rg_3 = 0;
       three++;
     }
     if (three % 2 == 0)
     {
       rg_3 = 125;
     }
     else
     {
       rg_3 = 0;
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
			servo[rg2] = (int) rg_2; //RIGHT
			servo[rg3] = (int) rg_3; //LEFT SERVO

		} // Close while (true) loop

} // Close task main()
