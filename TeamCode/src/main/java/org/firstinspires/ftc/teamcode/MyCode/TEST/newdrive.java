package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MyCode.Robot;

/**
 * Created by user on 1/19/18.
 */

public class newdrive extends LinearOpMode {


    Robot robot = new Robot();
    int counter = 0;


    @Override
    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap,telemetry);
        robot.ResetDriveEncoders();


        waitForStart();
        counter = 1;

        if(counter == 1){
            //robot.encoderDrive(.5,25,25,telemetry);
            sleep(5000);
            counter = 2;
        }
        if(counter == 2){
            sleep(10000);
            counter =3;
        }
        if(counter == 3);



    }

}
