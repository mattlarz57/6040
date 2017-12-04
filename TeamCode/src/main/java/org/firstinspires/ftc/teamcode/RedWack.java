package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by user on 12/02/17.
 */
@Autonomous
public class RedWack extends LinearOpMode {

    Robot robot = new Robot();
    int counter = 0;
    Robot.team Team = Robot.team.Red;
    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap,telemetry);





        waitForStart();
        counter =1;

        if(counter == 1){
            robot.WackJewel(Team);
            sleep(3000);
            counter =2;
        }
        if(counter == 2){
            robot.SetDrivePower(-.5);
            sleep(1000);
            robot.SetDrivePower(0);
            counter = 3;
        }



    }
}
