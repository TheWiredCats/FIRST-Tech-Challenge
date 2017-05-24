#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     right,         tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     right2,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     left,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     left2,         tmotorTetrix, openLoop)

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.



task main()
{


  waitForStart();


     motor[right] = 100;
     motor[right2]= 100;
     motor[left] =  100;
     motor[left2] = 100;
     wait1Msec(1000);


}
