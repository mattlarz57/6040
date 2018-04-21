package org.firstinspires.ftc.teamcode.MyCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 4/02/18.
 */
@TeleOp(group = "New")
public class NewTele extends OpMode {

    Robot robot = new Robot();
    boolean touchpressed;
    boolean autosqueeze;



    @Override
    public void init(){

        robot.initnew(hardwareMap);
        robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
        //robot.Camera.setPosition(RobotConstants.Camera_Forward);

    }
    @Override
    public void loop(){
        telemetry.addLine("hey");
        telemetry.update();
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        double drive = gamepad1.right_stick_y;
        double relic = gamepad2.right_stick_x;

        if(robot.Touch.isPressed()){
            touchpressed = true;
        }




        robot.BackRight.setPower((-turn + strafe - drive));
        robot.BackLeft.setPower((turn - strafe - drive));
        robot.FrontRight.setPower((-turn - strafe - drive));
        robot.FrontLeft.setPower((turn + strafe - drive));
        robot.relicArm.setPower(relic/1.3);


        if (gamepad2.right_trigger >= .1) {
            robot.NewSuckers(-1);

        }
        else if (gamepad2.left_trigger >= .1) {
            robot.NewSuckers(1);
        }
        else {
            robot.NewSuckers(0);
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


        /*if (gamepad2.a) {
            robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Closed_FUll);
            robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Close_Full);
        }
        else if (gamepad2.y){
                robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Closed_Ish);
                robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Close_Ish);
                autosqueeze = true;
        }
        else if (gamepad2.b) {
            robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Open);
            robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Open);
        }*/
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
            autosqueeze = false;
            touchpressed = false;
        } else {
            robot.Glyphter.setPower(0);
        }

        if(autosqueeze && touchpressed){
            if (gamepad2.a) {
                robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Closed_FUll);
                robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Close_Full);
            }
            else if (gamepad2.y){
                robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Closed_Ish);
                robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Close_Ish);
            }

        }
        else{
            if (gamepad2.y) {
                robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Closed_FUll);
                robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Close_Full);
            }
            else if (gamepad2.a){
                robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Closed_Ish);
                robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Close_Ish);
                autosqueeze = true;
            }

        }

        if (gamepad2.b) {
            robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Open);
            robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Open);
        }











        }

    }



