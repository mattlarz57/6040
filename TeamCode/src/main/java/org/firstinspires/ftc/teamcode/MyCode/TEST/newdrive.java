package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

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
        robot.Gyro.calibrate();
        while (robot.Gyro.isCalibrating()){}


        waitForStart();
        counter = 1;

        if(counter == 1){
           robot.DrivePID(.75f,25,telemetry,5,this);
            //robot.Drive(.75,50,telemetry,5);
            counter++;
        }
        if(counter == 2){
        }



    }

}
