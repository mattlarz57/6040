package org.firstinspires.ftc.teamcode.MyCode.BLUE;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.MyCode.*;
import org.firstinspires.ftc.teamcode.OtherFiles.*;
import org.firstinspires.ftc.teamcode.DogeExamples.*;

/**
 * Created by user on 12/02/17.
 */
public class BlueWack extends LinearOpMode {

    Robot robot = new Robot();
    int counter = 0;
    Robot.team Team = Robot.team.Blue;
    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap,telemetry);


        waitForStart();
        counter =1;

        if(counter == 1){
            robot.WackJewel(Team);
            sleep(1000);
            counter =2;
        }
        if(counter == 2){
            robot.SetDrivePower(.5);
            sleep(1000);
            robot.SetDrivePower(0);
            counter = 3;
        }



    }
}
