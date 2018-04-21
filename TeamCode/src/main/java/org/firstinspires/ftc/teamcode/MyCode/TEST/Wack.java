package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;

/**
 * Created by user on 1/25/18.
 */
public class Wack extends LinearOpMode {

    Robot robot = new Robot();
    int counter;
    JewelDetector jewelDetector = new JewelDetector();
    JewelDetector.JewelOrder jewelOutput = JewelDetector.JewelOrder.UNKNOWN;

    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap,telemetry);
        robot.Camera.setPosition(RobotConstants.Camera_Jewel);
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;


        waitForStart();

        jewelDetector.enable();
        while(opModeIsActive()){
            if(jewelDetector.getLastOrder() != JewelDetector.JewelOrder.UNKNOWN){
                jewelOutput = jewelDetector.getLastOrder();
                counter = 1;
            }
            if (counter == 1){
                robot.WackJewel(Robot.team.Red,jewelOutput);
                counter = 2;
            }
            telemetry.addData("Jewel", jewelOutput);
            telemetry.update();
        }


    }

}
