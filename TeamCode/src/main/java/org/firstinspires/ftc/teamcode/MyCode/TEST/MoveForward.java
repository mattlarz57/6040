package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.MyCode.*;
import org.firstinspires.ftc.teamcode.OtherFiles.*;
import org.firstinspires.ftc.teamcode.DogeExamples.*;

/**
 * Created by user on 11/30/17.
 */
@Autonomous
public class MoveForward extends LinearOpMode {


    Robot robot = new Robot();
    int counter = 0;
    Robot.team TeamColor = Robot.team.Blue;




    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap,telemetry);
        counter = 1;
        waitForStart();
        if (counter == 1) {
            robot.MoveTo(.25,25);
        }


    }





}
