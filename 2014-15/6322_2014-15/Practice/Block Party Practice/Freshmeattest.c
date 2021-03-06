#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     FL,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     BL,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     FR,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     BR,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     intake,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motorkk,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     motorK,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    claw,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickdriver.c";

task main()
{
int intaket = 0;
float clawv = 0;
while(true)
{

if(joystick.joy1_y1 < 10 && joystick.joy1_y1 >-10)
{
motor[BL] = 0;
motor[FL] = 0;
}
else
{
motor[BL] = joystick.joy1_y1;
motor[FL] = joystick.joy1_y1;
}

if(joystick.joy1_y2 < 10 && joystick.joy1_y2 >-10)
{
motor[BR] = 0;
motor[FR] = 0;
}
else
{
motor[BR] = joystick.joy1_y2;
motor[FR] = joystick.joy1_y2;
}

//intake
if(intaket == 0 && joy1Btn(2) == 1)
{
intaket = 1;
motor[intake] = 20;
}
else if(intaket == 1 && joy1Btn(2) == 0)
{
intaket = 2;
}
else if(intaket == 2 && joy1Btn(2) == 1)
{
intaket = 3;
motor[intake] = 0;
}
else if(intaket == 3 && joy1Btn(2) == 0)
{
intaket = 0;
}

//lift
if(joy1Btn(5) == 1 && joy1Btn(7) == 0)
{
motor[lift] = 100;
}
else if(joy1Btn(5) == 0 && joy1Btn(7) == 1)
{
motor[lift] = -100;
}
else
{
motor[lift] = 0;
}

//claw
if(joy1Btn(1) ==1)
{
clawv = 110;
}
if( joy1Btn(3) == 1 && joy1Btn(4) == 0)
{
clawv += .5;
}
else if(joy1Btn(3) == 0 && joy1Btn(4) == 1)
{
clawv -= .5;
}


if(clawv > 255)
{
clawv = 255;
}
else if(clawv < 0)
{
clawv = 0;
}

servo[claw] = (int)clawv;



}

}
