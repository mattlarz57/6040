package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MyCode.Robot;

/**
 * Created by user on 1/31/18.
 */
@Autonomous
public class OptimumSucker extends LinearOpMode {
    int counter;
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap,telemetry);
        counter = 0;



        waitForStart();
        counter = 1;


        while (opModeIsActive()){


            if(counter == 1){
                if(robot.SuckDone()){
                    counter ++;
                }
            }
        }
    }

}
