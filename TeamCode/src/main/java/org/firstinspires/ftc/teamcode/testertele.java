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

    @Override
    public void init(){
        robot.initialize(hardwareMap, telemetry);
        robot.SetParameters();


    }
    @Override
    public void loop(){ //this is what happens when you press the play button
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        double drive = gamepad1.right_stick_y;

        robot.BackRight.setPower((-turn + -strafe +drive));
        robot.BackLeft.setPower((-turn + strafe + drive));
        robot.FrontRight.setPower((turn + strafe + drive));
        robot.FrontLeft.setPower((turn + -strafe+ drive));








    }

}
