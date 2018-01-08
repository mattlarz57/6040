package org.firstinspires.ftc.teamcode.MyCode.RED;

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
@Autonomous(group = "Test")
public class RedWack extends LinearOpMode {

    Robot robot = new Robot();
    int counter = 0;
    Robot.team TeamColor = Robot.team.Red;
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap, telemetry);


        waitForStart();
        counter = 1;

        if (counter == 1) {
            //robot.DogeWack(TeamColor);
            sleep(1000);
            counter = 2;
        }
        if (counter == 2) {
           // robot.Move(-.5, 38);
            counter = 3;
        }

        if (counter == 3) {
            robot.jewelDetector.disable();
            stop();
        }


    }
}
