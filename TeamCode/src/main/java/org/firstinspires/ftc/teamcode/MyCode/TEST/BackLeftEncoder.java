package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.PushbotAutoDriveByEncoder_Linear;
import org.firstinspires.ftc.teamcode.MyCode.Robot;

/**
 * Created by user on 1/20/18.
 */
@Autonomous
public class BackLeftEncoder extends LinearOpMode {


    Robot robot = new Robot();
    int counter;


    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap, telemetry);
        counter = 1;

        waitForStart();

        if(counter == 1){
            robot.Drive(.5,25,telemetry);
            counter = 2;
        }

    }
}
