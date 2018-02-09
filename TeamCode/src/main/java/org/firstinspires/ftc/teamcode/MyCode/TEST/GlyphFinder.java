package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.MyCode.*;
import org.firstinspires.ftc.teamcode.OtherFiles.AutoTransitioner;
import org.opencv.core.Scalar;

/**
 * Created by user on 1/08/18.
 */
public class GlyphFinder extends OpMode {
    Robot robot = new Robot();
    RobotConstants robotConstants;
    GlyphDetector glyphDetector = new GlyphDetector();
    int counter;
    VuforiaLocalizer vuforia;
    int VuMarkSeen;
    Robot.team TeamColor = Robot.team.NotSensed;



    public void init() {


        telemetry.addLine("Initialization: Loading...");
        robot.initialize(hardwareMap, telemetry);
        glyphDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        glyphDetector.minScore = 1;
        glyphDetector.downScaleFactor = 0.3;
        glyphDetector.speed = GlyphDetector.GlyphDetectionSpeed.SLOW;
        glyphDetector.enable();

        telemetry.addLine("Initialization: Success");
        counter = 1;

    }


    @Override
    public void loop(){

        telemetry.addData("Glyph Pos X", robot.glyphDetector.getChosenGlyphOffset());
        telemetry.addData("Glyph Pos Offest String", robot.glyphDetector.getChosenGlyphPosition().toString());
        telemetry.addData("Glyph Pos Offest", robot.glyphDetector.getChosenGlyphPosition());


    }


    @Override
    public void stop(){
        robot.glyphDetector.disable();
    }
}
