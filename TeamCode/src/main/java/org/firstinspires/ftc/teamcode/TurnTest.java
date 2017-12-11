package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


//* Created by user on 12/07/17.


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
            robot.EncoderTurn("CounterClockWise",.75,90);
            counter ++;

            }

            }
        }





