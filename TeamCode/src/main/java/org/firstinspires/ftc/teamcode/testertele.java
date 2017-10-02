package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import javax.microedition.khronos.opengles.GL;

/**
 * Created by user on 2017-09-14.
 */
@TeleOp
public class testertele extends OpMode {
     DcMotor BR,BL,FR,FL,Glyphter;
     Servo RightGlyph,LeftGlyph;

    @Override
    public void init(){
        BR=hardwareMap.dcMotor.get("2a");
        BL= hardwareMap.dcMotor.get("2b");
        FR = hardwareMap.dcMotor.get("1a");
        FL = hardwareMap.dcMotor.get("1b");
        Glyphter = hardwareMap.dcMotor.get("3a");
        RightGlyph = hardwareMap.servo.get("RightGlyph");
        LeftGlyph = hardwareMap.servo.get("LeftGlyph");
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);


        RightGlyph.setPosition(.75);
        LeftGlyph.setPosition(.25);
    }
    @Override
    public void loop(){ //this is what happens when you press the play button
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        double drive = gamepad1.right_stick_y;

        BR.setPower((-turn + -strafe +drive));
        BL.setPower((-turn + strafe + drive));
        FR.setPower((turn + strafe + drive));
        FL.setPower((turn + -strafe+ drive));


        if (gamepad2.y){
            Glyphter.setPower(1);
        }
        else if(gamepad2.a){
            Glyphter.setPower(-1);
        }
        else{
            Glyphter.setPower(0);
        }

        if(gamepad2.b){
            RightGlyph.setPosition(.75);
            LeftGlyph.setPosition(.25);
        }
        else if (gamepad2.x){
            RightGlyph.setPosition(.99);
            LeftGlyph.setPosition(.01);
        }

    }

}
