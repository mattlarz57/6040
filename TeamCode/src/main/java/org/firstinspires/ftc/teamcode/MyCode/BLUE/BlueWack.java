package org.firstinspires.ftc.teamcode.MyCode.BLUE;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;

/**
 * Created by user on 2/08/18.
 */
@Autonomous
public class BlueWack extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException{
        Robot robot = new Robot();
        ElapsedTime elapsedTime = new ElapsedTime(0);
        JewelDetector jewelDetector = new JewelDetector();
        Robot.team TeamColor = Robot.team.Blue;
        JewelDetector.JewelOrder JewelOutput = JewelDetector.JewelOrder.UNKNOWN;

        int counter = 0;
        boolean disabled = false;

        telemetry.addLine("Initialization: Loading...");
        robot.initialize(hardwareMap, telemetry);
        robot.Camera.setPosition(RobotConstants.Camera_Jewel);
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;


        telemetry.update();
        telemetry.addLine("Initialization: Success");

        waitForStart();
        jewelDetector.enable();
        while (opModeIsActive()){
            if ( !disabled && jewelDetector.getLastOrder() != JewelDetector.JewelOrder.UNKNOWN) {
                JewelOutput = jewelDetector.getLastOrder();
                jewelDetector.disable();
                disabled = true;
                counter = 1;
            }
            if(counter == 1){
                try{
                    robot.WackJewel(TeamColor,JewelOutput);
                }
                catch (InterruptedException e){

                }
                counter ++;
            }
            if(counter == 2){
                robot.Drive(.35,60,telemetry,elapsedTime,3);
                counter ++;
            }
            if(counter == 3){
                robot.EncoderTurn(Robot.Direction.CounterClockWise,.35,90,elapsedTime,3);
                counter ++;
            }
            if(counter == 4){
                robot.Drive(.35,20,telemetry,elapsedTime,3);
                counter ++;
            }
            if(counter == 5){}
        }








    }


}
