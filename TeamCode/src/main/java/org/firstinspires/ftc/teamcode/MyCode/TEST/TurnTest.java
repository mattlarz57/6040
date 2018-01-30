package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.MyCode.*;
import org.firstinspires.ftc.teamcode.OtherFiles.*;
import org.firstinspires.ftc.teamcode.DogeExamples.*;

//* Created by user on 12/07/17.

@Autonomous
public class TurnTest extends LinearOpMode {

    Robot robot = new Robot();
    int counter;


    @Override
    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap,telemetry);
       // robot.SetParameters();




        counter =1;
        waitForStart();

        if(counter == 1){
            /*double ticks = RobotConstants.Tickspercm * 9.5 * Math.PI;
            int BRPos = Math.abs(robot.BackRight.getCurrentPosition());
            int BLPos = Math.abs(robot.BackLeft.getCurrentPosition());
            int FRPos = Math.abs(robot.FrontRight.getCurrentPosition());
            int FLPos = Math.abs(robot.FrontLeft.getCurrentPosition());

            int avg = (BRPos + BLPos + FRPos + FLPos) / 4;

            while(avg < ticks){
                robot.BackRight.setPower(-1);
                robot.BackLeft.setPower(-1);
                robot.FrontRight.setPower(1);
                robot.FrontLeft.setPower(1);
                sleep(100);
                BRPos = Math.abs(robot.BackRight.getCurrentPosition());
                BLPos = Math.abs(robot.BackLeft.getCurrentPosition());
                FRPos = Math.abs(robot.FrontRight.getCurrentPosition());
                FLPos = Math.abs(robot.FrontLeft.getCurrentPosition());
                avg = (BRPos + BLPos + FRPos + FLPos) / 4;
            }
            robot.SetDrivePower(0);
            counter = 2;
            */
            robot.EncoderTurn(Robot.Direction.CounterClockWise,.75,90);
            counter ++;

            }

            }
        }





