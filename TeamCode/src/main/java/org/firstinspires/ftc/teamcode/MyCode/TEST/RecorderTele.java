package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GhostExamples.CodeSharer;
import org.firstinspires.ftc.teamcode.GhostExamples.GhostRecorder;
import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;

/**
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 2/20/18.
 */
public class RecorderTele extends OpMode {
    Robot robot = new Robot();
    boolean AutoSucc;
    GhostRecorder relicplacer = new GhostRecorder();



    @Override
    public void init(){
        robot.initialize(hardwareMap,telemetry);

        robot.Camera.setPosition(RobotConstants.Camera_Forward);




    }


    @Override
    public void loop(){

        relicplacer.recordLeftBumper(gamepad2.left_bumper);
        relicplacer.recordRightBumper(gamepad2.right_bumper);
        relicplacer.recordDpadDown(gamepad2.dpad_down);
        relicplacer.recordDpadLeft(gamepad2.dpad_left);
        relicplacer.recordDpadRight(gamepad2.dpad_right);
        relicplacer.recordDpadUp(gamepad2.dpad_up);
        relicplacer.recordRightStickX(gamepad2.right_stick_x);

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

        relicplacer.update();

    }
    @Override
    public void stop(){
        CodeSharer CodeSharer=new CodeSharer(hardwareMap.appContext);
        CodeSharer.share(relicplacer.getString());
    }

}
