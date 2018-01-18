package org.firstinspires.ftc.teamcode.MyCode;

import com.google.gson.graph.GraphAdapterBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import javax.microedition.khronos.opengles.GL;

/**
 * Created by user on 2017-09-14.
 */
@TeleOp
public class FinalTele extends OpMode {
    Robot robot = new Robot();
    boolean AutoSucc;


    @Override
    public void init() {
        robot.initialize(hardwareMap, telemetry);
        //robot.SetParameters();


    }

    @Override
    public void loop() { //this is what happens when you press the play button
        telemetry.addData("AutoSucc On:", AutoSucc);
        telemetry.update();
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        double drive = gamepad1.right_stick_y;
        double relic = gamepad2.right_stick_x;
        int touchpressed;

        if (robot.Touch.isPressed()) {
            touchpressed = 0;
        } else {
            touchpressed = -1;
        }

        robot.BackRight.setPower((-turn + strafe - drive));
        robot.BackLeft.setPower((turn - strafe - drive));
        robot.FrontRight.setPower((-turn - strafe - drive));
        robot.FrontLeft.setPower((turn + strafe - drive));
        robot.relicArm.setPower(-relic / 2);


        if (gamepad2.right_trigger >= .1) {
            robot.GTR.setPower(touchpressed-0.07);
            robot.GBR.setPower(RobotConstants.Suckers_In-.137);
            robot.GTL.setPower(touchpressed-.12);
            robot.GBL.setPower(RobotConstants.Suckers_In-0);
        } else if (gamepad2.left_trigger >= .1) {
            robot.Suckers(RobotConstants.Suckers_Out);
        } else {
            robot.Suckers(RobotConstants.Suckers_Stay);
        }

      if (gamepad1.dpad_left) {
            robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Left);
        } else if (gamepad1.dpad_right) {
            robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Right);
        }



        if (gamepad1.dpad_up) {
            robot.Jeweler1.setPosition(RobotConstants.Jeweler1_Up);
        } else if (gamepad1.dpad_down) {
            robot.Jeweler1.setPosition(RobotConstants.Jeweler1_Down);
        }


        if (gamepad2.a) {
            robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Close);
            robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Close);

        } else if (gamepad2.b) {
            robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Open);
            robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Open);
        }
        if (gamepad2.dpad_up) {
            robot.BigRelicBack.setPosition(RobotConstants.BigRelicBack_In);
            robot.BigRelicFront.setPosition(RobotConstants.BigRelicFront_In);
        }
        else if (gamepad2.dpad_down) {
            robot.BigRelicFront.setPosition(RobotConstants.BigRelicFront_Out);
            robot.BigRelicBack.setPosition(RobotConstants.BigRelicBack_Out);
        }
        if (gamepad2.dpad_right) {
            robot.relicSmall.setPosition(RobotConstants.Small_Relic_Close);
        } else if (gamepad2.dpad_left) {
            robot.relicSmall.setPosition(RobotConstants.Small_Relic_Open);
        }

        if (gamepad2.left_stick_y > .1) {
            robot.Glyphter.setPower(1);
        } else if (gamepad2.left_stick_y < -.1) {
            robot.Glyphter.setPower(-1);
        } else {
            robot.Glyphter.setPower(0);
        }

        if (gamepad2.y) {
            AutoSucc = true;

        } else if (gamepad2.x) {
            AutoSucc = false;
        }

        if (AutoSucc && robot.Dist.getLightDetected() > 0.015) {
            robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Close);
            robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Close);
            AutoSucc = false;


        }

    }

}
