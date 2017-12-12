package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by user on 12/02/17.
 */
public class RedWack extends LinearOpMode {

    Robot robot = new Robot();
    int counter = 0;
    Robot.team TeamColor = Robot.team.Red;
    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap,telemetry);





        waitForStart();
        counter =1;

        if(counter == 1){
            robot.WackJewel(TeamColor);
            sleep(1000);
            counter =2;
        }
        if(counter == 2){
            robot.Move(-.5,38);
            counter = 3;
        }



    }
}
