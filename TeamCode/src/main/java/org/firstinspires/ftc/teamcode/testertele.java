package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by user on 2017-09-14.
 */
@TeleOp
public class testertele extends OpMode {
     DcMotor BR,BL,FR,FL;
    @Override
    public void init(){
        BR=hardwareMap.dcMotor.get("2a");
        BL= hardwareMap.dcMotor.get("2b");
        FR = hardwareMap.dcMotor.get("1a");
        FL = hardwareMap.dcMotor.get("1b");
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop(){
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        double drive = gamepad1.right_stick_y;

        BR.setPower((-turn + -strafe +drive)/3);
        BL.setPower((turn + strafe + drive)/3);
        FR.setPower((-turn + strafe + drive)/3);
        FL.setPower((turn + -strafe+ drive)/3);

    }

}
