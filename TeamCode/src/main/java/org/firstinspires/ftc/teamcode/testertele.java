package org.firstinspires.ftc.teamcode;

import com.google.gson.graph.GraphAdapterBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import javax.microedition.khronos.opengles.GL;

/**
 * Created by user on 2017-09-14.
 */
@TeleOp
public class testertele extends OpMode {
    Robot robot = new Robot();

    public DcMotor BackRight, BackLeft, FrontRight, FrontLeft, Glyphter;
    public Servo RightGlyph, LeftGlyph, Grabber1, Grabber2;
    public CRServo Sucker1, Sucker2;




    @Override
    public void init(){
        // boolean doneinit = robot.initialize(hardwareMap,telemetry);
        //while(doneinit == false){}
        BackRight=hardwareMap.dcMotor.get("2a");
        BackLeft= hardwareMap.dcMotor.get("2b");
        FrontRight = hardwareMap.dcMotor.get("1a");
        FrontLeft = hardwareMap.dcMotor.get("1b");
        Glyphter = hardwareMap.dcMotor.get("3a");
        RightGlyph = hardwareMap.servo.get("RightGlyph");
        LeftGlyph = hardwareMap.servo.get("LeftGlyph");
        Grabber1 = hardwareMap.servo.get("Grabber1");
        Grabber2 = hardwareMap.servo.get("Grabber2");
        Sucker1 = hardwareMap.crservo.get("Sucker1");
        Sucker2 = hardwareMap.crservo.get("Sucker2");
        Glyphter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bno055IMU= hardwareMap.get(BNO055IMU.class,"IMU");

        RightGlyph.setPosition(0);
        LeftGlyph.setPosition(.6);
        robot.SetParameters();




    }
    @Override
    public void loop(){ //this is what happens when you press the play button
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        double drive = gamepad1.right_stick_y;

        BackRight.setPower((-turn + -strafe +drive));
        BackLeft.setPower((-turn + strafe + drive));
        FrontRight.setPower((turn + strafe + drive));
         FrontLeft.setPower((turn + -strafe+ drive));


        //if (gamepad2.y){Glyphter.setPower(1);}
       // else if(gamepad2.a){Glyphter.setPower(-1);}
       // else{Glyphter.setPower(0);}

        if(gamepad2.b){
            RightGlyph.setPosition(.75);
            LeftGlyph.setPosition(.25);
        }
        else if (gamepad2.x){
            RightGlyph.setPosition(.99);
            LeftGlyph.setPosition(.01);}

        if(gamepad1.right_bumper){
            Grabber1.setPosition(1);
        }
        else if (gamepad1.left_bumper){
            Grabber1.setPosition(0);
        }
        if(gamepad1.dpad_up){
            Grabber2.setPosition(0);
        }
        else if(gamepad1.dpad_down){
            Grabber2.setPosition(1);
        }

/*
        if(gamepad2.right_bumper){
            RightGlyph.setPosition(.99);
            LeftGlyph.setPosition(.01);
            Glyphter.setTargetPosition(500);
            Glyphter.setPower(.5);
            if(Glyphter.getCurrentPosition() == Glyphter.getTargetPosition()){
                Glyphter.setPower(0);
            }
        }
       else if(gamepad2.left_bumper){
            RightGlyph.setPosition(.75);
            LeftGlyph.setPosition(.25);;
            Glyphter.setTargetPosition(0);
            Glyphter.setPower(-.5);
            if(Glyphter.getCurrentPosition() == Glyphter.getTargetPosition()){
                Glyphter.setPower(0);
            }


        }
        */
        if(gamepad2.right_trigger >.1){
            Sucker1.setPower(1);
            Sucker2.setPower(-1);

        }
        else if (gamepad2.left_trigger>.1){
            Sucker2.setPower(-1);
            Sucker1.setPower(1);
        }
        else{
            Sucker1.setPower(0);
            Sucker2.setPower(0);
        }






    }

}
