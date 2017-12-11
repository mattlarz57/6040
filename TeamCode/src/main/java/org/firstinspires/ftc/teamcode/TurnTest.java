package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


 //* Created by user on 12/07/17.

@Autonomous
public class TurnTest extends LinearOpMode {

    Robot robot = new Robot();
    int counter;


    @Override
    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap,telemetry);
        robot.SetParameters();


        counter =1;
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Heading" , robot.getheading());

            if (counter == 1){
                robot.DegreeTurn("Right",.5,270);
            }
        }


    }

}
