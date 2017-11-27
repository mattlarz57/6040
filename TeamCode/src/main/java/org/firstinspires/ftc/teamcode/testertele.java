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
       robot.initialize(hardwareMap,telemetry);
        robot.SetParameters();


    }
    @Override
    public void loop(){ //this is what happens when you press the play button
        //telemetry.addData("Heading:",robot.getheading());
        telemetry.addData("X",robot.getorientaion()[0]);
        telemetry.addData("Y",robot.getorientaion()[1]);
        telemetry.addData("Z",robot.getorientaion()[2]);
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        double drive = gamepad1.right_stick_y;
        double glyph = gamepad2.left_stick_y;
        double relic = gamepad2.right_stick_x;
        int touchpressed;

        if(robot.Touch.isPressed()){
            touchpressed = 0;
        }
        else{
            touchpressed=1;
        }

        robot.BackRight.setPower((-turn + -strafe +drive));
        robot.BackLeft.setPower((-turn + strafe + drive));
        robot.FrontRight.setPower((turn + strafe + drive));
        robot.FrontLeft.setPower((turn + -strafe+ drive));
        robot.Glyphter.setPower(-glyph);
        robot.relicArm.setPower(-relic /2);


        if(gamepad2.right_trigger>=.1){
            robot.GTR.setPower(touchpressed);
            robot.GBR.setPower(RobotConstants.Suckers_In);
            robot.GTL.setPower(touchpressed);
            robot.GBL.setPower(RobotConstants.Suckers_In);
        }
        else if (gamepad2.left_trigger>=.1){
            robot.GBL.setPower(RobotConstants.Suckers_Out);
            robot.GBR.setPower(RobotConstants.Suckers_Out);
            robot.GTL.setPower(RobotConstants.Suckers_Out);
            robot.GTR.setPower(RobotConstants.Suckers_Out);
        }
        else{
            robot.GBL.setPower(RobotConstants.Suckers_Stay);
            robot.GBR.setPower(RobotConstants.Suckers_Stay);
            robot.GTL.setPower(RobotConstants.Suckers_Stay);
            robot.GTR.setPower(RobotConstants.Suckers_Stay);
        }

        if(gamepad1.dpad_left){
            robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Left);
        }
        else if(gamepad1.dpad_right){
            robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Right);
        }
        else {
            robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
        }

        if(gamepad1.dpad_up){
            robot.Jeweler1.setPosition(RobotConstants.Jeweler1_Up);
        }
        else if(gamepad1.dpad_down){
            robot.Jeweler1.setPosition(RobotConstants.Jeweler1_Down);
        }

        if(gamepad2.a){
            robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Close);  //21
            robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Close);//79

        }
        else if (gamepad2.b){
            robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Open);
            robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Open);
        }
        if (gamepad2.dpad_up){
            robot.relicBig.setPosition(RobotConstants.Big_Relic_Up);
        }
        else if (gamepad2.dpad_down){
            robot.relicBig.setPosition(RobotConstants.Big_Relic_Down);
        }
        if (gamepad2.dpad_right){
            robot.relicSmall.setPosition(RobotConstants.Small_Relic_Close);
        }
        else if (gamepad2.dpad_left){
            robot.relicSmall.setPosition(RobotConstants.Small_Relic_Open);
        }

    }

}
