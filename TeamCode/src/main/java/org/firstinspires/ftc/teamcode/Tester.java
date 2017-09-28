package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

/**
 * Created by user on 2017-08-29.
 */
@Autonomous
public class Tester extends LinearOpMode {

    Robot robot = new Robot();
    double[] angles;
    VuforiaTrackable relicTemplate;


    @Override
    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap, telemetry);
        relicTemplate = robot.vuforia(hardwareMap,telemetry);


        AutoTransitioner.transitionOnStop(this,"testertele");

        waitForStart();

        while(opModeIsActive()) {
            double[] angles = robot.getorientaion();
            RelicRecoveryVuMark vumark = robot.getvuMark(relicTemplate);
            // telemetry.addLine("X (heading): "+ Math.round(angles[0]));
            //  telemetry.addLine("Y: (heading) "+ Math.round(angles[1]));
            telemetry.addLine("heading "+Math.round(robot.getheading()));
            telemetry.addData("Time Elapsed: ", Math.round(getRuntime()));
            telemetry.addData("vuMark" , vumark);
            telemetry.update();
        }

        //robot.Move(.1,25);
        //sleep(10000);
        //robot.Sideways("Right",.2,10);




    }



}

