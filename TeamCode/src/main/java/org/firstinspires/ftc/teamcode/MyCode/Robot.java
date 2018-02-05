package org.firstinspires.ftc.teamcode.MyCode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Set;

/**
 * Created by user on 2017-08-29.
 */

public class Robot {
    public enum team {
        Red, Blue, NotSensed
    }
    public enum Direction{
        CounterClockWise, ClockWise
    }


    RobotConstants robotConstants = new RobotConstants();


    public DcMotor BackRight, BackLeft, FrontRight, FrontLeft, Glyphter, relicArm;
    public Servo  SqueezerR, SqueezerL, relicSmall, BigRelic, Jeweler1, Jeweler2, Camera ;
    public CRServo GBR, GBL, GTR, GTL;
    public BNO055IMU bno055IMU;
    public ModernRoboticsTouchSensor Touch;
    public JewelDetector jewelDetector;
    public GlyphDetector glyphDetector;
    public OpticalDistanceSensor Dist;


    Telemetry t;


    public boolean initialize(HardwareMap hardwareMap, Telemetry telemetry) {

        t = telemetry;
        Jeweler1 = hardwareMap.servo.get("Jeweler1");
        Jeweler2 = hardwareMap.servo.get("Jeweler2");
        BackRight = hardwareMap.dcMotor.get("BR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        FrontLeft = hardwareMap.dcMotor.get("FL");
        Glyphter = hardwareMap.dcMotor.get("Glyphter");
        relicArm = hardwareMap.dcMotor.get("Relic");
        GTL = hardwareMap.crservo.get("GTL");
        GBL = hardwareMap.crservo.get("GBL");
        GTR = hardwareMap.crservo.get("GTR");
        GBR = hardwareMap.crservo.get("GBR");
        SqueezerL = hardwareMap.servo.get("SqueezerL");
        SqueezerR = hardwareMap.servo.get("SqueezerR");
        BigRelic = hardwareMap.servo.get("BigRelic");
        relicSmall = hardwareMap.servo.get("SmallRelic");
        Camera = hardwareMap.servo.get("Camera");
        GTR.setDirection(DcMotorSimple.Direction.REVERSE);
        GBR.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Touch = hardwareMap.get(ModernRoboticsTouchSensor.class, "Touch");
        Dist = hardwareMap.get(OpticalDistanceSensor.class, "Distance");





        SqueezerL.setPosition(robotConstants.SqueezerL_Open);
        SqueezerR.setPosition(robotConstants.SqueezerR_Open);
        Suckers(RobotConstants.Suckers_Stay);
        BigRelic.setPosition(0);
        relicSmall.setPosition(0);

        Jeweler2.setPosition(robotConstants.Jeweler2_Left);
        Jeweler1.setPosition(robotConstants.Jeweler1_Up);
        Camera.setPosition(robotConstants.Camera_Jewel);
        relicSmall.setPosition(robotConstants.Small_Relic_Close);
        BigRelic.setPosition(robotConstants.BigRelicIn);




        //SetParameters();






        return true;


    }

   /* public VuforiaTrackable vuforia(HardwareMap hardwareMap, Telemetry telemetry){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        return relicTemplate;

    }
    */



    public void GlyphDetectorInit(HardwareMap hardwareMap){
        glyphDetector = new GlyphDetector();
        glyphDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        glyphDetector.minScore =1;
        glyphDetector.downScaleFactor = 0.3;
        glyphDetector.speed = GlyphDetector.GlyphDetectionSpeed.BALANCED;
        glyphDetector.enable();
    }

    public void ResetDriveEncoders() {
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (BackRight.getCurrentPosition() != 0 || BackLeft.getCurrentPosition() != 0 || FrontRight.getCurrentPosition() != 0 || FrontLeft.getCurrentPosition() != 0) {

        }

        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void DriveMotorMode(DcMotor.RunMode mode){
        BackRight.setMode(mode);
        BackLeft.setMode(mode);
        FrontLeft.setMode(mode);
        FrontRight.setMode(mode);
    }

    public void SetDrivePower(double power) {
        BackRight.setPower(power);
        BackLeft.setPower(power);
        FrontRight.setPower(power);
        FrontLeft.setPower(power);
    }

    public void Move(double power, double centimeters) {
        ResetDriveEncoders();
        double ticks = centimeters * robotConstants.Tickspercm;
        int BRPos = Math.abs(BackRight.getCurrentPosition());
        int BLPos = Math.abs(BackLeft.getCurrentPosition());
        int FRPos = Math.abs(FrontRight.getCurrentPosition());
        int FLPos = Math.abs(FrontLeft.getCurrentPosition());

        double avg = ((BRPos + BLPos + FRPos + FLPos) / 4);

        while (avg < ticks) {
            SetDrivePower(power);
            BRPos = Math.abs(BackRight.getCurrentPosition());
            BLPos = Math.abs(BackLeft.getCurrentPosition());
            FRPos = Math.abs(FrontRight.getCurrentPosition());
            FLPos = Math.abs(FrontLeft.getCurrentPosition());
            avg = ((BRPos + BLPos + FRPos + FLPos) / 4);
        }
        SetDrivePower(0);
    }
    public void MoveTo(double power, long centimeters, Telemetry telemetry){
        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ticks = centimeters * robotConstants.Tickspercm;
        int rounded = (int)(ticks);
        BackRight.setTargetPosition(rounded);
        BackLeft.setTargetPosition(rounded);
        FrontRight.setTargetPosition(rounded);
        FrontLeft.setTargetPosition(rounded);

        DriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetDrivePower(power);
        int BRCurr,BLCurr,FRCurr,FLCurr;
        BRCurr = BackRight.getCurrentPosition();
        BLCurr = BackLeft.getCurrentPosition();
        FRCurr = FrontRight.getCurrentPosition();
        FLCurr = FrontLeft.getCurrentPosition();
        while (FrontRight.isBusy() || FrontLeft.isBusy()|| BackRight.isBusy()||BackLeft.isBusy()){
            telemetry.addData("FRPos: ", FRCurr);
            telemetry.addData("FLPos: ", FLCurr);
            telemetry.addData("BRPos: ",BRCurr);
            telemetry.addData("BLPos", BRCurr);
            BRCurr = BackRight.getCurrentPosition();
            BLCurr = BackLeft.getCurrentPosition();
            FRCurr = FrontRight.getCurrentPosition();
            FLCurr = FrontLeft.getCurrentPosition();
            telemetry.update();
        }
        SetDrivePower(0);
        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void Backwards(double power, double centimeters) {
        ResetDriveEncoders();
        double ticks = centimeters * robotConstants.Tickspercm;
        int BRPos = (BackRight.getCurrentPosition());
        int BLPos = (BackLeft.getCurrentPosition());
        int FRPos = (FrontRight.getCurrentPosition());
        int FLPos = (FrontLeft.getCurrentPosition());

        double avg = ((BRPos + BLPos + FRPos + FLPos) / 4);

        while (avg > ticks) {
            SetDrivePower(-power);
            BRPos = (BackRight.getCurrentPosition());
            BLPos = (BackLeft.getCurrentPosition());
            FRPos = (FrontRight.getCurrentPosition());
            FLPos = (FrontLeft.getCurrentPosition());
            avg = ((BRPos + BLPos + FRPos + FLPos) / 4);
        }

        SetDrivePower(0);
    }

    public void Sideways(String direction, double power, double centimeters) {
        ResetDriveEncoders();
        double ticks = centimeters * robotConstants.Sidewaystickspercm;
        double SidewaysSwitcher;
        int BRPos = Math.abs(BackRight.getCurrentPosition());
        int BLPos = Math.abs(BackLeft.getCurrentPosition());
        int FRPos = Math.abs(FrontRight.getCurrentPosition());
        int FLPos = Math.abs(FrontLeft.getCurrentPosition());
        double avg = ((BRPos + BLPos + FRPos + FLPos) / 4);
        if (direction == "Right") {
            SidewaysSwitcher = 1;
        } else if (direction == "Left") {
            SidewaysSwitcher = -1;
        } else {
            SidewaysSwitcher = 0;
        }
        while (avg < ticks) {
            BackRight.setPower(power * SidewaysSwitcher);
            BackLeft.setPower(-1 * power * SidewaysSwitcher);
            FrontRight.setPower(-1 * power * SidewaysSwitcher);
            FrontLeft.setPower(power * SidewaysSwitcher);
            BRPos = Math.abs(BackRight.getCurrentPosition());
            BLPos = Math.abs(BackLeft.getCurrentPosition());
            FRPos = Math.abs(FrontRight.getCurrentPosition());
            FLPos = Math.abs(FrontLeft.getCurrentPosition());
            avg = (BRPos + BLPos + FRPos + FLPos) / 4;
        }

        SetDrivePower(0);
        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void EncoderTurn(Direction Direction , double power, double degrees, ElapsedTime elapsedTime, double timeout) throws InterruptedException{
        ResetDriveEncoders();
        double startime = elapsedTime.seconds();
        double ticks = degrees * robotConstants.cmPerDegree * robotConstants.Tickspercm;
        if(Direction == Robot.Direction.CounterClockWise){
            int BRPos = Math.abs(BackRight.getCurrentPosition());
            int BLPos = Math.abs(BackLeft.getCurrentPosition());
            int FRPos = Math.abs(FrontRight.getCurrentPosition());
            int FLPos = Math.abs(FrontLeft.getCurrentPosition());
            int avg = (BRPos + BLPos + FRPos + FLPos) / 4;
            double currentime = elapsedTime.seconds();

            while(avg < ticks  && currentime - startime < timeout){
                BackRight.setPower(power);
                BackLeft.setPower(-power);
                FrontRight.setPower(power);
                FrontLeft.setPower(-power);
                BRPos = Math.abs(BackRight.getCurrentPosition());
                BLPos = Math.abs(BackLeft.getCurrentPosition());
                FRPos = Math.abs(FrontRight.getCurrentPosition());
                FLPos = Math.abs(FrontLeft.getCurrentPosition());
                avg = (BRPos + BLPos + FRPos + FLPos) / 4;
                currentime = elapsedTime.seconds();

            }
            SetDrivePower(0);
            ResetDriveEncoders();

        }

        else if (Direction == Robot.Direction.ClockWise){
            int BRPos = Math.abs(BackRight.getCurrentPosition());
            int BLPos = Math.abs(BackLeft.getCurrentPosition());
            int FRPos = Math.abs(FrontRight.getCurrentPosition());
            int FLPos = Math.abs(FrontLeft.getCurrentPosition());
            int avg = (BRPos + BLPos + FRPos + FLPos) / 4;
            double currenttime = elapsedTime.seconds();


            while(avg < ticks && currenttime - startime < timeout){
                BackRight.setPower(-power);
                BackLeft.setPower(power);
                FrontRight.setPower(-power);
                FrontLeft.setPower(power);
                BRPos = Math.abs(BackRight.getCurrentPosition());
                BLPos = Math.abs(BackLeft.getCurrentPosition());
                FRPos = Math.abs(FrontRight.getCurrentPosition());
                FLPos = Math.abs(FrontLeft.getCurrentPosition());
                avg = (BRPos + BLPos + FRPos + FLPos) / 4;
                currenttime = elapsedTime.seconds();

            }
            SetDrivePower(0);
            DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

    }

    public void BetterTurn(Direction direction, double power, double degrees){
        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double ticks = degrees * robotConstants.cmPerDegree * robotConstants.Tickspercm;
        int rounded = (int)(ticks);
        if (direction == Direction.ClockWise) {
            BackRight.setTargetPosition(-rounded);
            BackLeft.setTargetPosition(rounded);
            FrontRight.setTargetPosition(-rounded);
            FrontLeft.setTargetPosition(rounded);
        }
        else if (direction == Direction.CounterClockWise) {
            BackRight.setTargetPosition(rounded);
            BackLeft.setTargetPosition(-rounded);
            FrontRight.setTargetPosition(rounded);
            FrontLeft.setTargetPosition(-rounded);
        }

        DriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetDrivePower(power);

        int BRCurr,BLCurr,FRCurr,FLCurr;
        BRCurr = BackRight.getCurrentPosition();
        BLCurr = BackLeft.getCurrentPosition();
        FRCurr = FrontRight.getCurrentPosition();
        FLCurr = FrontLeft.getCurrentPosition();
        while (!withindistance(BRCurr,rounded,50)||!withindistance(BLCurr,rounded,50)||!withindistance(FRCurr,rounded,50) || !withindistance(FLCurr,rounded,50)){
            BRCurr = BackRight.getCurrentPosition();
            BLCurr = BackLeft.getCurrentPosition();
            FRCurr = FrontRight.getCurrentPosition();
            FLCurr = FrontLeft.getCurrentPosition();
        }
        SetDrivePower(0);
        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }



    public void WackJewel(team TeamColor, JewelDetector.JewelOrder orientation)  throws  InterruptedException{
        Jeweler1.setPosition(RobotConstants.Jeweler1_Down);
        Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
        Thread.sleep(1000);


        if (TeamColor == team.Blue) {
            if (orientation == JewelDetector.JewelOrder.RED_BLUE) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Left);
                Thread.sleep(500);
            } else if (orientation == JewelDetector.JewelOrder.BLUE_RED) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Right);
                Thread.sleep(500);
            }
            Jeweler1.setPosition(robotConstants.Jeweler1_Up);
            Thread.sleep(400);
            Jeweler2.setPosition(robotConstants.Jeweler2_Left);
        }
        else if (TeamColor == team.Red){
            if (orientation == JewelDetector.JewelOrder.RED_BLUE) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Right);
                Thread.sleep(500);
            } else if (orientation == JewelDetector.JewelOrder.BLUE_RED) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Left);
                Thread.sleep(500);
            }
            Jeweler1.setPosition(robotConstants.Jeweler1_Up);
            Thread.sleep(400);
            Jeweler2.setPosition(robotConstants.Jeweler2_Left);
        }

    }

    public void DogeWack(team TeamColor) throws InterruptedException{
        Jeweler1.setPosition(RobotConstants.Jeweler1_Down);
        Thread.sleep(775);
        Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
        Thread.sleep(2000);

        if (TeamColor == team.Blue) {
            if (jewelDetector.getLastOrder() == JewelDetector.JewelOrder.RED_BLUE) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Left);
                Thread.sleep(500);
            } else if (jewelDetector.getLastOrder() == JewelDetector.JewelOrder.BLUE_RED) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Right);
                Thread.sleep(500);
            }
            Jeweler1.setPosition(robotConstants.Jeweler1_Up);
            Thread.sleep(400);
            Jeweler2.setPosition(robotConstants.Jeweler2_Left);
        }
        else if (TeamColor == team.Red){
            if (jewelDetector.getLastOrder() == JewelDetector.JewelOrder.RED_BLUE) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Right);
                Thread.sleep(500);
            } else if (jewelDetector.getLastOrder() == JewelDetector.JewelOrder.BLUE_RED) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Left);
                Thread.sleep(500);
            }
            Jeweler1.setPosition(robotConstants.Jeweler1_Up);
            Thread.sleep(400);
            Jeweler2.setPosition(robotConstants.Jeweler2_Left);
        }
    }


    public void Suckers(double power){
        GTR.setPower(power);
        GBR.setPower(power);
        GBL.setPower(power);
        GTL.setPower(power);

    }


    public boolean withindistance(int current, int target, int threshold ){
        int error = target - current;

        if (Math.abs(error)<= threshold){
            return true;
        }
        else{
            return false;
        }
    }
    public void encoderDrive(double speed, double leftcm,double rightcm,Telemetry telemetry){

        ResetDriveEncoders();
        while(BackLeft.getCurrentPosition() != 0){}
        int leftticks = (int)(robotConstants.Tickspercm*leftcm);
        int rightticks = (int)(robotConstants.Tickspercm*rightcm);

        BackRight.setTargetPosition(rightticks);
        BackLeft.setTargetPosition(leftticks);
        FrontLeft.setTargetPosition(leftticks);
        FrontRight.setTargetPosition(rightticks);

        DriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        BackLeft.setPower(speed);
        BackRight.setPower(speed);
        FrontLeft.setPower(speed);
        FrontRight.setPower(speed);

        while ( BackLeft.isBusy() || BackRight.isBusy() || FrontLeft.isBusy() || FrontRight.isBusy()){
            telemetry.addData("BR",BackRight.getCurrentPosition());
            telemetry.addData("FR",FrontRight.getCurrentPosition());
            telemetry.addData("BL", BackLeft.getCurrentPosition());
            telemetry.addData("FL", FrontLeft.getCurrentPosition());
            telemetry.update();
        }
        SetDrivePower(0);
        ResetDriveEncoders();


    }


    public void Drive(double speed, double centimeters, Telemetry telemetry, ElapsedTime elapsedTime, double timeout){

        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int ticks = (int)(robotConstants.Tickspercm *centimeters);
        double starttime = elapsedTime.seconds();

        BackRight.setTargetPosition(ticks);
        BackLeft.setTargetPosition(ticks);
        FrontRight.setTargetPosition(ticks);
        FrontLeft.setTargetPosition(ticks);


        DriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetDrivePower(speed);
        int BRPos = (BackRight.getCurrentPosition());
        int BLPos = (BackLeft.getCurrentPosition());
        int FRPos = (FrontRight.getCurrentPosition());
        int FLPos = (FrontLeft.getCurrentPosition());
        double currentime = elapsedTime.seconds();


        while((Math.abs(BRPos-ticks) > 75 || Math.abs(BLPos-ticks) > 75 || Math.abs(FRPos-ticks) > 75 ||Math.abs(FLPos-ticks) > 75)  && currentime - starttime < timeout) {
            telemetry.addData("posBR", BackRight.getCurrentPosition());
            telemetry.addData("PosBL", BackLeft.getCurrentPosition());
            telemetry.addData("PosFR", FrontRight.getCurrentPosition());
            telemetry.addData("PosFL", FrontLeft.getCurrentPosition());
            currentime = elapsedTime.seconds();

            telemetry.update();
            BRPos = (BackRight.getCurrentPosition());
            BLPos = (BackLeft.getCurrentPosition());
            FRPos = (FrontRight.getCurrentPosition());
            FLPos = (FrontLeft.getCurrentPosition());




        }
        /*if(BackLeft.getCurrentPosition() == ticks && BackRight.getCurrentPosition() == ticks
            && FrontLeft.getCurrentPosition() == ticks && FrontRight.getCurrentPosition() == ticks){
            SetDrivePower(0);
            DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        */

        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }

    public void Turn(Direction direction, double speed, double degrees, Telemetry telemetry){
        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int ticks = (int)(degrees*robotConstants.Tickspercm*robotConstants.cmPerDegree);

        if(direction == Direction.ClockWise){
            FrontLeft.setTargetPosition(ticks);
            FrontRight.setTargetPosition(-ticks);
            BackLeft.setTargetPosition(ticks);
            BackRight.setTargetPosition(-ticks);
        }
        else if (direction == Direction.CounterClockWise){
            FrontLeft.setTargetPosition(-ticks);
            FrontRight.setTargetPosition(ticks);
            BackLeft.setTargetPosition(-ticks);
            BackRight.setTargetPosition(ticks);
        }
        DriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetDrivePower(speed);

        int BRPos = (BackRight.getCurrentPosition());
        int BLPos = (BackLeft.getCurrentPosition());
        int FRPos = (FrontRight.getCurrentPosition());
        int FLPos = (FrontLeft.getCurrentPosition());


        while(Math.abs(BRPos-ticks) > 75 || Math.abs(BLPos-ticks) > 75 || Math.abs(FRPos-ticks) > 75 ||Math.abs(FLPos-ticks) > 75 ){
            telemetry.addData("posBR", BackRight.getCurrentPosition());
            telemetry.addData("PosBL", BackLeft.getCurrentPosition());
            telemetry.addData("PosFR", FrontRight.getCurrentPosition());
            telemetry.addData("PosFL", FrontLeft.getCurrentPosition());

            telemetry.update();
            BRPos = (BackRight.getCurrentPosition());
            BLPos = (BackLeft.getCurrentPosition());
            FRPos = (FrontRight.getCurrentPosition());
            FLPos = (FrontLeft.getCurrentPosition());
        }
        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public boolean SuckDone(){

        int ODSCount = 0;
        int touchpressed;
        SetDrivePower(.35);

        while(ODSCount < 2){

            if(Touch.isPressed()){touchpressed = 0; }
            else{touchpressed = 1;}

            GTR.setPower(touchpressed);
            GTL.setPower(touchpressed);
            GBR.setPower(robotConstants.Suckers_In);
            GBL.setPower(robotConstants.Suckers_In);

            if(Dist.getLightDetected() >= .015){
                SqueezerL.setPosition(robotConstants.SqueezerL_Close);
                SqueezerR.setPosition(robotConstants.SqueezerR_Close);
                ODSCount ++;
            }
            else{
                SqueezerL.setPosition(robotConstants.SqueezerL_Open);
                SqueezerR.setPosition(robotConstants.SqueezerR_Open);
            }

        }
        SetDrivePower(0);
        Suckers(robotConstants.Suckers_Stay);
        SqueezerL.setPosition(robotConstants.SqueezerL_Open);
        SqueezerR.setPosition(robotConstants.SqueezerR_Open);
        return true;
    }


}







