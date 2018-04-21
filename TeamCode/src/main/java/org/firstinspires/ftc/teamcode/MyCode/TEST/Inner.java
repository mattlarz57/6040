package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MyCode.Robot;

/**
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 2/17/18.
 */
public class Inner extends LinearOpMode {


    Robot robot = new Robot();
    int counter;

    @Override
    public void runOpMode(){

        robot.initialize(hardwareMap,telemetry);


        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Distance", robot.Inside.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

    }
}
