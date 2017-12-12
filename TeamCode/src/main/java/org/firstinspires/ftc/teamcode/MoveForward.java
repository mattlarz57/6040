package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by user on 11/30/17.
 */
public class MoveForward extends LinearOpMode {


    Robot robot = new Robot();
    int counter = 0;





    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap,telemetry);
        counter = 1;
        waitForStart();
        if (counter == 1) {
            robot.Move(.15, 50);
        }


    }





}
