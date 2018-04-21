package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        glyphDetector.minScore = 4;
        glyphDetector.downScaleFactor = 0.3;
        glyphDetector.speed = GlyphDetector.GlyphDetectionSpeed.SLOW;
        glyphDetector.rotateMat = true;
        glyphDetector.enable();


        telemetry.addLine("Initialization: Success");
        counter = 1;

    }


    @Override
    public void loop() {


        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        double drive = gamepad1.right_stick_y;
        robot.BackRight.setPower((-turn + strafe - drive));
        robot.BackLeft.setPower((turn - strafe - drive));
        robot.FrontRight.setPower((-turn - strafe - drive));
        robot.FrontLeft.setPower((turn + strafe - drive));
        if (glyphDetector.isFoundRect()) {

            telemetry.addData("Glyph Pos X", glyphDetector.getChosenGlyphOffset());
           // telemetry.addData("Glyph Pos Offest String", glyphDetector.getChosenGlyphPosition().toString());
            telemetry.addData("Glyph Pos Offest", glyphDetector.getChosenGlyphPosition());
//            telemetry.addData("width: ", glyphDetector.getWidth());
        }

    }


    @Override
    public void stop(){
        glyphDetector.disable();
    }
}
