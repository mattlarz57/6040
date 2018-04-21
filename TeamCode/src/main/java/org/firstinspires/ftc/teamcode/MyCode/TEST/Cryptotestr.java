package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 3/26/18.
 */
public class Cryptotestr extends LinearOpMode {


    Robot robot = new Robot();
    ElapsedTime runtime = new ElapsedTime(0);

    @Override
    public void runOpMode() throws InterruptedException{


        robot.initialize(hardwareMap,telemetry);
        CryptoboxDetector cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        cryptoboxDetector.downScaleFactor = 0.4;
        cryptoboxDetector.setColor("Red");


        waitForStart();


        cryptoboxDetector.enable();

        runtime.reset();

        List<Double> distances = new ArrayList<>();
        while (opModeIsActive() && runtime.seconds() < 5) {
            if (cryptoboxDetector.getDistanceToMove() > -7.5 && cryptoboxDetector.getDistanceToMove() < 7.5) {
                distances.add(cryptoboxDetector.getDistanceToMove());
            }
            telemetry.addData("Distance to Move: ", cryptoboxDetector.getDistanceToMove());
            telemetry.update();
        }

        double sum = 0;

        for (double distance: distances) {
            sum += distance;
        }

        double distanceToMove = sum / distances.size();

        if (distanceToMove < 7.5 && distanceToMove > -7.5) {
            if (distanceToMove > 0) {
                robot.Sideways("Right",1,distanceToMove * RobotConstants.in2cm);
            } else if (distanceToMove < 0) {
                robot.Sideways("Left",1,distanceToMove * RobotConstants.in2cm);
            }

            telemetry.addData("Distance Moved: ", (int) distanceToMove);
        } else {
            if (distanceToMove == 100) {
                telemetry.addData("Distance Moved: ", "No Cryptobox");
            } else {
                telemetry.addData("Distance Moved: ", "Double Stacked Box");
            }
        }

        telemetry.update();

        cryptoboxDetector.disable();

        sleep(5000);
    }





    }



