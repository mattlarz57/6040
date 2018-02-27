package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;

/**
 * Created by user on 1/31/18.
 */
public class OptimumSucker extends LinearOpMode {
    int counter;
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap,telemetry);
        counter = 0;



        waitForStart();
        counter = 1;
        double avg =0;
        int touchpressed;

        while (opModeIsActive()){
            if(robot.Touch.isPressed()){touchpressed = 0;}
            else{touchpressed = 1;}


            if(counter == 1){

                ElapsedTime runtime = new ElapsedTime(0);
                while(avg < 85* RobotConstants.Tickspercm && robot.Inside.getDistance(DistanceUnit.CM)>3){

                    robot.SetDrivePower(.5);
                    avg = (robot.FrontLeft.getCurrentPosition() +
                            robot.FrontRight.getCurrentPosition() +
                            robot.BackLeft.getCurrentPosition() +
                            robot.BackRight.getCurrentPosition())/4;
                    telemetry.addData("avg: ", avg);
                    telemetry.addData("time: ", runtime.seconds());

                }
                counter ++;

            }
            if(counter == 2){
                robot.SetDrivePower(0);
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Close);
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Close);
                robot.GTR.setPower(touchpressed);
                robot.GBR.setPower(RobotConstants.Suckers_In);
                robot.GTL.setPower(touchpressed);
                robot.GBL.setPower(RobotConstants.Suckers_In);

            }
            if(counter == 3){


            }
        }
    }
}
