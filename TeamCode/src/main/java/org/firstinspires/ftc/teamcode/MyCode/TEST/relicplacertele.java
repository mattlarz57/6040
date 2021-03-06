package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.google.gson.graph.GraphAdapterBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GhostExamples.GhostController;
import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;

import javax.microedition.khronos.opengles.GL;

/**
 * Created by user on 2017-09-14.
 */
public class relicplacertele extends OpMode {
    Robot robot = new Robot();
    boolean AutoSucc;




    @Override
    public void init() {
        robot.initialize(hardwareMap, telemetry);
        //robot.SetParameters();
        robot.Camera.setPosition(RobotConstants.Camera_Forward);



    }

    @Override
    public void loop() { //this is what happens when you press the play button
        telemetry.addData("AutoSucc On:", AutoSucc);
        //telemetry.addData("Closeness: " , robot.Dist.getLightDetected());
        telemetry.update();
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        double drive = gamepad1.right_stick_y;
        double relic = gamepad2.right_stick_x;
        int touchpressed;

        if(robot.Touch.isPressed()){touchpressed = 0;}
        else{touchpressed = 1;}


        robot.BackRight.setPower((-turn + strafe - drive));
        robot.BackLeft.setPower((turn - strafe - drive));
        robot.FrontRight.setPower((-turn - strafe - drive));
        robot.FrontLeft.setPower((turn + strafe - drive));
        robot.relicArm.setPower(relic / 1.3);


        if (gamepad2.right_trigger >= .1) {
            robot.GTR.setPower(touchpressed);
            robot.GBR.setPower(RobotConstants.Suckers_In);
            robot.GTL.setPower(touchpressed);
            robot.GBL.setPower(RobotConstants.Suckers_In);

        }
        else if (gamepad2.left_trigger >= .1) {
            robot.Suckers(RobotConstants.Suckers_Out);
        }
        else {
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
        if (gamepad2.dpad_down) {
            robot.BigRelic.setPosition(RobotConstants.BigRelicOut);
        }
        else if (gamepad2.dpad_up) {
            robot.BigRelic.setPosition(RobotConstants.BigRelicIn);//RobotConstants.BigRelicBack_Out);
        }
        while(gamepad1.right_trigger >= .1){
            robot.relicSmall.setPosition(robot.relicSmall.getPosition() + .00025);

        }
        while (gamepad1.left_trigger >= .1){
            robot.relicSmall.setPosition(robot.relicSmall.getPosition() - .00025);
        }

        while(gamepad2.right_bumper){
            robot.relicSmall.setPosition(robot.relicSmall.getPosition() + .00025);

        }
        if (gamepad2.dpad_right) {
            robot.relicSmall.setPosition(RobotConstants.Small_Relic_Close);
        } else if (gamepad2.dpad_left) {
            robot.relicSmall.setPosition(RobotConstants.Small_Relic_Open);
        }
        else if (gamepad2.left_bumper){
            robot.relicSmall.setPosition(RobotConstants.Small_Relic_Grab);
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
            robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Open);
            robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Open);

        } else if (gamepad2.x) {
            AutoSucc = false;
        }

        if (AutoSucc && robot.Inside.getDistance(DistanceUnit.CM) < 15) {
            robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Close);
            robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Close);
            AutoSucc = false;


        }
        if(gamepad1.right_bumper){
            GhostController ghostController = new GhostController("rx:1.0 175 bl:true 2 bl:false 325 rx:0.0 du:true 3 du:false br:true 3 br:false 175 br:true 1 br:false 75 br:true 1 br:false 75 br:true 1 br:false 75\n" +
                    "rx:-1.0 175 dr:true 2 dr:false rx:-1.0 325 rx:0.0 10\n");
            while (ghostController.areInstructionsLeft()){
                ghostController.update();
                double recordedrelic = ghostController.rightStickX();
                robot.relicArm.setPower(recordedrelic/1.3);
                if (ghostController.dpadRight()) {
                    robot.relicSmall.setPosition(RobotConstants.Small_Relic_Close);
                } else if (ghostController.dpadLeft()) {
                    robot.relicSmall.setPosition(RobotConstants.Small_Relic_Open);
                }
                else if (ghostController.bumperLeft()){
                    robot.relicSmall.setPosition(RobotConstants.Small_Relic_Grab);
                }
                while(ghostController.bumperRight()){
                    robot.relicSmall.setPosition(robot.relicSmall.getPosition() + .00025);

                }
                if (ghostController.dpadDown()) {
                    robot.BigRelic.setPosition(RobotConstants.BigRelicOut);
                }
                else if (ghostController.dpadUp()) {
                    robot.BigRelic.setPosition(RobotConstants.BigRelicIn);//RobotConstants.BigRelicBack_Out);
                }


            }
        }



    }

}
