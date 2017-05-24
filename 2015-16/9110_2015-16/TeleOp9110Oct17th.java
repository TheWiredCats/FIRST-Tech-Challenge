package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by David on 10/15/2015.
 */
public class TeleOp9110Oct17th extends OpMode{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor rightArm;
    DcMotor leftArm;
    DcMotor belt;

    final double BUCKET_OPEN_POSITION = 0.0;
    final double BUCKET_CLOSED_POSITION = 0.5;
    //final double LEFT_OPEN_POSITION = 0.0;
    //final double LEFT_CLOSED_POSITION = 0.5;
    //final double RIGHT_OPEN_POSITION = 0.0;
    //final double RIGHT_CLOSED_POSITION = 0.5;
    //Servo leftDoor;
    //Servo rightDoor;
    Servo bucket;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("left_drive");
        frontRight = hardwareMap.dcMotor.get("right_drive");
        backLeft = hardwareMap.dcMotor.get("backleft_drive");
        backRight = hardwareMap.dcMotor.get("backright_drive");
        rightArm = hardwareMap.dcMotor.get("rightarm_drive");
        leftArm = hardwareMap.dcMotor.get("leftarm_drive");
        belt = hardwareMap.dcMotor.get("belt_drive");
        bucket = hardwareMap.servo.get("bucket");
        //leftDoor = hardwareMap.servo.get("left_door");
        //rightDoor = hardwareMap.servo.get("right_door");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        leftArm.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // These are the wheels //
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.left_stick_y;

        frontLeft.setPower(leftY);
        backLeft.setPower(leftY);
        frontRight.setPower(rightY);
        backRight.setPower(rightY);

        // These are the arms //
        if(gamepad1.y) {
            leftArm.setPower(0.1);
            rightArm.setPower(0.1);
        }
        else if(gamepad1.a){
            leftArm.setPower(-0.1);
            rightArm.setPower(-0.1);
        }
        else {
            leftArm.setPower(0);
            rightArm.setPower(0);
        }

        // These are the servos //
        /*if(gamepad1.dpad_left){
            bucket.setPosition(BUCKET_OPEN_POSITION);
        }
        else if(gamepad1.dpad_right){
            bucket.setPosition(BUCKET_CLOSED_POSITION);
        }
        //if(gamepad1.dpad_up){
            //leftDoor.setPosition(LEFT_OPEN_POSITION);
            //rightDoor.setPosition(RIGHT_OPEN_POSITION);
        }
        //else if(gamepad1.dpad_down){
            //leftDoor.setPosition(LEFT_CLOSED_POSITION);
            //rightDoor.setPosition(RIGHT_CLOSED_POSITION);
        //}
        */


    }
}
