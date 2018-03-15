package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;

/**
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 3/06/18.
 */
@Autonomous
public class TurnTest extends LinearOpMode{

    int counter = 0;

    Robot robot = new Robot();



    @Override
    public void runOpMode() throws InterruptedException{



        robot.initialize(hardwareMap,telemetry);

        counter =1;
        while(robot.Gyro.isCalibrating()){}
        waitForStart();

        if (counter ==1){
            robot.Drive(.5,20,telemetry,3);
            counter ++;
        }
        if(counter == 2) {
            robot.GyroTurn(Robot.Direction.ClockWise, RobotConstants.HalfRotation,.5,telemetry,3);
            counter ++;
        }

        if(counter == 3){};

    }






}
