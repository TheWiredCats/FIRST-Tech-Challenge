4#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     FrontLeft,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     BackLeft,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     FrontRright,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     BackRight,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     Sweep,tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     frrfr,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     fffff,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    claw,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)

#include "joystickdriver.c"
task main()
{
  int Sweeps = 0; //sweep integer

	while (true)
  {
    getJoystickSettings (joystick);


    if (joystick.joy1_y1 > -10 && joystick.joy1_y1 < 10)
    {
    		motor [FrontRight] = 0;
    		motor [BackRight] = 0;
  	}
  	if (joystick.joy1_y2 > -10 && joystick.joy1_y2 < 10)
  	{
  		motor [FrontLeft] = 0;
  		motor [BackLeft] = 0;
    }


    if (joystick.joy1_y1 < -10 && joystick.joy1_y1 > 10)
    {
    	motor [FrontLeft] = joystick.joy1_y1;
    	motor [BackLeft] = joystick.joy1_y1;
    }

    if (joystick.joy1_y2 < -10 && joystick.joy1_y2 > 10)
    {
    	motor [FrontRight] = joystick.joy1_y2;
    	motor [BackRight] = joystick.joy1_y2;

    }

    if (joy2Btn(2) == 0 && Sweeps == 0)
    {
    	servo [Sweep] = 0;
    	Sweeps = 1;

    }
    /*if else (joy2Btn(2) == 1 && Sweeps == 1)
   {
     	servo [Sweep] = 180;
     	Sweeps = 2;
    }

    if else (joy2Btn(2) == 1 && Sweeps == 2)
    {
    	servo [Sweep] = 0;
    	Sweeps = 0;
    }
*/
  }}
