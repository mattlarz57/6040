package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.MyCode.*;
import org.firstinspires.ftc.teamcode.OtherFiles.*;
import org.firstinspires.ftc.teamcode.DogeExamples.*;


public class SampleTeleOp extends OpMode {
    Robot robot = new Robot();




    @Override
    public void init(){
        robot.initialize(hardwareMap,telemetry);

    }



    @Override
    public void loop(){

        
    }


}
