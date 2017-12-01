package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by user on 11/30/17.
 */

public class MoveForward extends LinearOpMode {


    Robot robot = new Robot();
    int counter = 0;



    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addLine("Initialization: Loading...");
        robot.initialize(hardwareMap,telemetry);
        robot.ResetDriveEncoders();
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Initialization: Success");

        waitForStart();
        counter = 1;

        if(counter == 1){
            robot.Move(1,100);
            counter = 2;
        }




    }





}
