package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by STUDENT on 10/26/2015.
 */
public class LENNY extends OpMode{

        DcMotor leftarm;
        DcMotor rightarm;
        DcMotor frontleft;
        DcMotor frontright;
        DcMotor backleft;
        DcMotor backright;
        DcMotor belt_drive;
        Servo bucket;
        Servo climber;
        //Servo button;

final double BUCKET_LEFT_POSITION = 0.60;
final double BUCKET_RIGHT_POSITION = 0.80;
final double BUCKET_STAND_POSITION = 0.70;
final double Climber_Stand = 1.00 ;
final double Climber_Dump = 0.05;
//final double Button_Stand = 0.5;
//final double Button_Left = 1.0;
//final double Button_Right = 0.0;

@Override
public void init() {
        frontleft = hardwareMap.dcMotor.get("frontleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");
        belt_drive = hardwareMap.dcMotor.get("belt_drive");
        bucket = hardwareMap.servo.get("bucket");
        climber = hardwareMap.servo.get("climber");
        leftarm = hardwareMap.dcMotor.get("leftarm");
        rightarm = hardwareMap.dcMotor.get("rightarm");
        //button = hardwareMap.servo.get("button");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        leftarm.setDirection(DcMotor.Direction.REVERSE);

        }

@Override
public void loop() {
        // These are the wheels
        float leftY = gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;
        float armsL = gamepad2.left_stick_y;
        float armsR = gamepad2.right_stick_y;

        frontleft.setPower(leftY);
        frontright.setPower(rightY);
        backleft.setPower(leftY);
        backright.setPower(rightY);
        leftarm.setPower(armsL);
        rightarm.setPower(armsR);

        //Bucket Movement++
        if(gamepad2.left_bumper==true && gamepad2.right_bumper==false){
        bucket.setPosition(BUCKET_LEFT_POSITION);
        }
        else if(gamepad2.right_bumper==true && gamepad2.left_bumper==false){
        bucket.setPosition(BUCKET_RIGHT_POSITION);
        }
        else if(gamepad2.left_bumper==false && gamepad2.right_bumper== false){
        bucket.setPosition(BUCKET_STAND_POSITION);
        }

        if (gamepad1.a==true && gamepad1.b==false && gamepad1.x==false) {

                belt_drive.setPower(0.65);
        }
        else if(gamepad1.a==false && gamepad1.b==false && gamepad1.x==true){

                belt_drive.setPower(-1.0);

                }
        else if(gamepad1.a==false && gamepad1.b==true && gamepad1.x==false){

                belt_drive.setPower(0.0);

        }



        if(gamepad2.a==true && gamepad2.b==false){

                climber.setPosition(Climber_Dump);

            }
        else if(gamepad2.a==false && gamepad2.b==true){

                climber.setPosition(Climber_Stand);

            }
        else if(gamepad2.a==false && gamepad2.b==false){

                climber.setPosition(Climber_Stand);

           }
        //if(gamepad1.left_bumper==true && gamepad1.right_bumper==false){
          //      button.setPosition(Button_Left);
        //}
        //else if(gamepad1.right_bumper==true && gamepad1.left_bumper==false){
          //      button.setPosition(Button_Right);
    //    }
       // else if(gamepad1.left_bumper==false && gamepad1.right_bumper== false){
         //       button.setPosition(Button_Stand);
        //}
         }



        }


