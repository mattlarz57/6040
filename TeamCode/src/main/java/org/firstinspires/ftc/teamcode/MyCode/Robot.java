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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
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
import org.firstinspires.ftc.teamcode.MyCode.TEST.PIDController;

import java.util.Set;

import static java.lang.Thread.sleep;

/**
 * Created by user on 2017-08-29.
 */

public class Robot {
    public enum team {
        Red, Blue, NotSensed
    }

    public enum Direction {
        CounterClockWise, ClockWise
    }


    RobotConstants robotConstants = new RobotConstants();


    public DcMotor BackRight, BackLeft, FrontRight, FrontLeft, Glyphter, relicArm, RightSpinner,LeftSpinner;
    public Servo SqueezerR, SqueezerL, relicSmall, BigRelic, Jeweler1, Jeweler2, Camera, NewSqueezerR, NewSqueezerL;
    public CRServo GBR, GBL, GTR, GTL;
    public ModernRoboticsTouchSensor Touch;
    public JewelDetector jewelDetector;
    public ModernRoboticsI2cRangeSensor Side, Inside,BackSide;
    public ModernRoboticsI2cGyro Gyro;





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
        Side = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Side");
        BackSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"BackSide");
        Inside = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Inside");
        I2cAddr SideRangeAdress = I2cAddr.create8bit(0x1a);
        I2cAddr InsideRangeAdress = I2cAddr.create8bit(0x2a);
        I2cAddr BackSideRangeAdress = I2cAddr.create8bit(0x3a);
        Side.setI2cAddress(SideRangeAdress);
        Inside.setI2cAddress(InsideRangeAdress);
        BackSide.setI2cAddress(BackSideRangeAdress);
        Gyro = hardwareMap.get(ModernRoboticsI2cGyro.class,"Gyro");
        Gyro.setI2cAddress(I2cAddr.create8bit(0x1c));




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
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        //SetParameters();

        Gyro.calibrate();
        return true;
    }

    public void  initnew(HardwareMap hardwareMap){
        Jeweler1 = hardwareMap.servo.get("Jeweler1");
        Jeweler2 = hardwareMap.servo.get("Jeweler2");
        BackRight = hardwareMap.dcMotor.get("BR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        FrontLeft = hardwareMap.dcMotor.get("FL");
        Glyphter = hardwareMap.dcMotor.get("Glyphter");
        relicArm = hardwareMap.dcMotor.get("Relic");
        RightSpinner = hardwareMap.dcMotor.get("RightSpinner");
        LeftSpinner = hardwareMap.dcMotor.get("LeftSpinner");
        NewSqueezerL = hardwareMap.servo.get("NewSqueezerL");
        NewSqueezerR = hardwareMap.servo.get("NewSqueezerR");
        BigRelic = hardwareMap.servo.get("BigRelic");
        relicSmall = hardwareMap.servo.get("SmallRelic");
        Camera = hardwareMap.servo.get("Camera");
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        RightSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        Side = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Side");
        BackSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"BackSide");
        I2cAddr SideRangeAdress = I2cAddr.create8bit(0x1a);
        I2cAddr BackSideRangeAdress = I2cAddr.create8bit(0x3a);
        Side.setI2cAddress(SideRangeAdress);
        BackSide.setI2cAddress(BackSideRangeAdress);
        Gyro = hardwareMap.get(ModernRoboticsI2cGyro.class,"Gyro");
        Gyro.setI2cAddress(I2cAddr.create8bit(0x1c));
        Touch = hardwareMap.get(ModernRoboticsTouchSensor.class,"Touch");



        NewSqueezerL.setPosition(robotConstants.NewSqueezerL_Open);
        NewSqueezerR.setPosition(robotConstants.NewSqueezerR_Open);
        NewSuckers(0);
        BigRelic.setPosition(0);
        relicSmall.setPosition(0);
        Jeweler2.setPosition(robotConstants.Jeweler2_Left);
        Jeweler1.setPosition(robotConstants.Jeweler1_Up);
        //Camera.setPosition(robotConstants.Camera_Jewel);
        relicSmall.setPosition(robotConstants.Small_Relic_Close);
        BigRelic.setPosition(robotConstants.BigRelicIn);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





    }

    public void NewSuckers(double power){
        RightSpinner.setPower(power);
        LeftSpinner.setPower(power);
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

    public void DriveMotorMode(DcMotor.RunMode mode) {
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

    public void EncoderTurn(Direction Direction, double power, double degrees, double timeout) throws InterruptedException {
        boolean timedout =false;
        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime time = new ElapsedTime();
        int ticks = (int) (robotConstants.Tickspercm * degrees * RobotConstants.cmPerDegree);
        double FRPos = FrontRight.getCurrentPosition(),
                FLPos = FrontLeft.getCurrentPosition(),
                BRPos = BackRight.getCurrentPosition(),
                BLPos = BackLeft.getCurrentPosition();

        if (Direction == Robot.Direction.CounterClockWise) {
            BackRight.setTargetPosition(ticks);
            BackLeft.setTargetPosition(-ticks);
            FrontRight.setTargetPosition(ticks);
            FrontLeft.setTargetPosition(-ticks);

            DriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setPower(power);
            FrontLeft.setPower(-power);
            BackLeft.setPower(-power);
            BackRight.setPower(power);
            double currenttime;


            while (!timedout && (Math.abs(ticks - FRPos) > 75 || Math.abs(-ticks - FLPos) > 75 || Math.abs(ticks - BRPos) > 75 || Math.abs(-ticks - BLPos) > 75)) {
                FRPos = FrontRight.getCurrentPosition();
                FLPos = FrontLeft.getCurrentPosition();
                BRPos = BackRight.getCurrentPosition();
                BLPos = BackLeft.getCurrentPosition();

                currenttime = time.seconds();
                if (timeout < currenttime) {timedout=true;}
            }
            DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if (Direction == Robot.Direction.ClockWise) {

            BackRight.setTargetPosition(-ticks);
            BackLeft.setTargetPosition(ticks);
            FrontRight.setTargetPosition(-ticks);
            FrontLeft.setTargetPosition(ticks);

            DriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            SetDrivePower(power);

            double currenttime;

            while (!timedout && (Math.abs(-ticks - FRPos) > 75 || Math.abs(ticks - FLPos) > 75 || Math.abs(-ticks - BRPos) > 75 || Math.abs(ticks - BLPos) > 75)) {
                FRPos = FrontRight.getCurrentPosition();
                FLPos = FrontLeft.getCurrentPosition();
                BRPos = BackRight.getCurrentPosition();
                BLPos = BackLeft.getCurrentPosition();

                currenttime = time.seconds();
                if (timeout < currenttime) {timedout = true;}
            }
            DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


    public void WackJewel(team TeamColor, JewelDetector.JewelOrder orientation) throws InterruptedException {
        Jeweler1.setPosition(RobotConstants.Jeweler1_Down);
        Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
        sleep(500);


        if (TeamColor == team.Blue) {
            if (orientation == JewelDetector.JewelOrder.BLUE_RED) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Left);
                sleep(250);
            } else if (orientation == JewelDetector.JewelOrder.RED_BLUE) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Right);
                sleep(250);
            }
            Jeweler1.setPosition(robotConstants.Jeweler1_Up);
            sleep(400);
            Jeweler2.setPosition(robotConstants.Jeweler2_Left);
        } else if (TeamColor == team.Red) {
            if (orientation == JewelDetector.JewelOrder.BLUE_RED) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Right);
                sleep(250);
            } else if (orientation == JewelDetector.JewelOrder.RED_BLUE) {
                Jeweler2.setPosition(robotConstants.Jeweler2_Left);
                sleep(250);
            }
            Jeweler1.setPosition(robotConstants.Jeweler1_Up);
            sleep(400);
            Jeweler2.setPosition(robotConstants.Jeweler2_Left);
        }

    }


    public void Suckers(double power) {
        GTR.setPower(power);
        GBR.setPower(power);
        GBL.setPower(power);
        GTL.setPower(power);

    }


    public void Drive(double speed, double centimeters, Telemetry telemetry, double timeout) {

        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean timedout = false;
        ElapsedTime time = new ElapsedTime();
        int ticks = (int) (robotConstants.Tickspercm * centimeters);

        BackRight.setTargetPosition(ticks);
        BackLeft.setTargetPosition(ticks);
        FrontRight.setTargetPosition(ticks);
        FrontLeft.setTargetPosition(ticks);

        DriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetDrivePower(speed);

        double currenttime = time.seconds();

        while (!timedout && (FrontLeft.isBusy() || FrontRight.isBusy() || BackLeft.isBusy() || BackRight.isBusy())/*  && (currenttime  < timeout) */) {
            telemetry.addData("posBR", BackRight.getCurrentPosition());
            telemetry.addData("PosBL", BackLeft.getCurrentPosition());
            telemetry.addData("PosFR", FrontRight.getCurrentPosition());
            telemetry.addData("PosFL", FrontLeft.getCurrentPosition());
            telemetry.addData("time", currenttime);

            telemetry.update();

            currenttime = time.seconds();
            if (timeout < currenttime) {
                timedout = true;
            }
        }
        SetDrivePower(0);
        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.update();
        telemetry.addLine("Drive Complete");
    }


    public double DriveWithSuck(double speed, double distance, Telemetry telemetry, double timeout) {


        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean timedout = false;
        ElapsedTime time = new ElapsedTime();
        int ticks = (int) (robotConstants.Tickspercm * distance);

        BackRight.setTargetPosition(ticks);
        BackLeft.setTargetPosition(ticks);
        FrontRight.setTargetPosition(ticks);
        FrontLeft.setTargetPosition(ticks);

        DriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        SetDrivePower(speed);

        double currenttime = time.seconds();

        while (!timedout && (FrontLeft.isBusy() || FrontRight.isBusy() || BackLeft.isBusy() || BackRight.isBusy())/*  && (currenttime  < timeout) */)

        {
            telemetry.addData("posBR", BackRight.getCurrentPosition());
            telemetry.addData("PosBL", BackLeft.getCurrentPosition());
            telemetry.addData("PosFR", FrontRight.getCurrentPosition());
            telemetry.addData("PosFL", FrontLeft.getCurrentPosition());
            telemetry.addData("time", currenttime);

            telemetry.update();

            currenttime = time.seconds();
            if(Inside.getDistance(DistanceUnit.CM) < 15){
                SqueezerL.setPosition(RobotConstants.SqueezerL_Close);
                SqueezerR.setPosition(RobotConstants.SqueezerR_Close);

            }
            if (timeout < currenttime) {
                timedout = true;
            }
        }
        double avg = (BackRight.getCurrentPosition()+
         BackLeft.getCurrentPosition()+
         FrontRight.getCurrentPosition()+
         FrontLeft.getCurrentPosition())/4;

        SqueezerR.setPosition(RobotConstants.SqueezerR_Close);
        SqueezerL.setPosition(RobotConstants.SqueezerL_Close);

        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return avg;

    }

    public boolean GyroTurn(Direction direction,double degrees, double speed,Telemetry telemetry,double timeout) {
        double y = 0;
        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElapsedTime ttime = new ElapsedTime();
        while(FrontLeft.getCurrentPosition() >5 || FrontRight.getCurrentPosition() >5 || BackLeft.getCurrentPosition() >5 || BackRight.getCurrentPosition() >5){
            telemetry.update();
            telemetry.addLine("resetting");}
        DriveMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Gyro.isCalibrating()) {
            telemetry.update();
            telemetry.addLine("claibrating");}


        if (direction == Direction.ClockWise) {
            while (Math.abs(y) < (Math.abs(degrees)) && (ttime.seconds() < timeout)) {
                y = Gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                BackRight.setPower(-speed);
                FrontRight.setPower(-speed);
                BackLeft.setPower(speed);
                FrontLeft.setPower(speed);
                telemetry.addData("Y : ", y);
            }
            SetDrivePower(0);
            DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Gyro.calibrate();

            return true;
        } else if (direction == Direction.CounterClockWise) {
            while (Math.abs(y) < (Math.abs(degrees)) && ttime.seconds() < timeout) {
                y = Gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                BackRight.setPower(speed);
                FrontRight.setPower(speed);
                BackLeft.setPower(-speed);
                FrontLeft.setPower(-speed);
                telemetry.addData("Y : ", y);
            }
            SetDrivePower(0);
            DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Gyro.calibrate();
            return true;
        }
        else{
            return false;
        }
    }




    public void PIDTurn(Direction direction,int degrees, float MaxPowah,Telemetry telemetry,float timeout, LinearOpMode opMode ) throws InterruptedException{
        Gyro.calibrate();
        ElapsedTime time = new ElapsedTime(0);
        float heading,MV;
        PIDController PID = new PIDController(10,0);
        boolean done;
        double donecounter=0;
       if (direction == Direction.ClockWise){
            degrees = -degrees;
        }




        PID.setPIDParam(PIDController.PID_SETPOINT,degrees);
        PID.setPIDParam(PIDController.PID_LIMITHIGH,MaxPowah*10);
        PID.setPIDParam(PIDController.PID_LIMITLOW,MaxPowah * -10);
        PID.setPIDParam(PIDController.PID_KP,.4f);
        PID.setPIDParam(PIDController.PID_KI,.0004f);
        PID.setPIDParam(PIDController.PID_KD,16f);
        //PID.setPIDParam(PIDController.PID_DEADBAND, .6f);




        while(Gyro.isCalibrating()){
            telemetry.addData("Gyro Heading: ", Gyro.getAngularOrientation(AxesReference.EXTRINSIC,AxesOrder.ZXY,AngleUnit.DEGREES).firstAngle);
            telemetry.update();
        }



        MV = 1;
        time.reset();
        done = false;

        while( opMode.opModeIsActive() && time.seconds() < timeout && !done){
            heading = Gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
            MV = PID.doPID((int)heading);
            FrontRight.setPower(MV/10);
            BackRight.setPower(MV/10);
            FrontLeft.setPower(-1*MV/10);
            BackLeft.setPower(-1*MV/10);

            telemetry.addData("Speed: ", MV/10);
            telemetry.addData("Gyro Heading: ", heading);
            telemetry.addData("Time: ", time.seconds());
            telemetry.update();
            sleep(50);
            if(Math.abs((int)heading - degrees) < 2){
                donecounter++;
                if(donecounter>10){
                    done = true;
                }
            }
            else{
                donecounter = 0;
            }

        }
        SetDrivePower(0);


    }

    public void DriveStraightWithGyroAndEncoder(float speed, double distance, Telemetry telemetry, double timeout, LinearOpMode opmode) throws InterruptedException{
        PIDController gyro = new PIDController(0,0);
        PIDController encoders = new PIDController((int)(distance * RobotConstants.Tickspercm),0);
        boolean done = false;
        float GMV,EMV, heading, encoderdistance, RightPower,LeftPower;
        ElapsedTime time = new ElapsedTime(0);


        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro.setPIDParam(PIDController.PID_SETPOINT,0);
        gyro.setPIDParam(PIDController.PID_LIMITHIGH,speed*10);
        gyro.setPIDParam(PIDController.PID_LIMITLOW,speed*-10);
        gyro.setPIDParam(PIDController.PID_KP,.3f);
        gyro.setPIDParam(PIDController.PID_DEADBAND, .6f);

        encoders.setPIDParam(PIDController.PID_SETPOINT,(int)(distance * RobotConstants.Tickspercm));
        encoders.setPIDParam(PIDController.PID_LIMITHIGH,speed*10);
        encoders.setPIDParam(PIDController.PID_LIMITLOW,speed*-10);
        encoders.setPIDParam(PIDController.PID_KP,.025f);

        encoders.setPIDParam(PIDController.PID_DEADBAND,.05f);

        Gyro.calibrate();
        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(Gyro.isCalibrating() || FrontLeft.getCurrentPosition() != 0){}
        time.reset();

        while(opmode.opModeIsActive() && time.seconds() < timeout && !done){
            encoderdistance = ((FrontLeft.getCurrentPosition() + FrontRight.getCurrentPosition() + BackLeft.getCurrentPosition() + BackRight.getCurrentPosition())/4);
            heading = Gyro.getAngularOrientation(AxesReference.EXTRINSIC,AxesOrder.ZXY,AngleUnit.DEGREES).firstAngle;

            EMV =encoders.doPID((int)encoderdistance);
            GMV =gyro.doPID((int)heading);

            if((int)heading > 0){
                RightPower = EMV -GMV;
                LeftPower = EMV + GMV;
            }
            else if((int)heading < 0){
                RightPower = EMV +GMV;
                LeftPower = EMV - GMV;
            }
            else{
                RightPower = EMV;
                LeftPower = EMV;
            }

            FrontRight.setPower(RightPower/10);
            BackRight.setPower(RightPower/10);
            BackLeft.setPower(LeftPower/10);
            FrontLeft.setPower(LeftPower/10);

            telemetry.addData("Right Power: ", RightPower);
            telemetry.addData("Left Power: ", LeftPower);
            telemetry.addData("time: ", time.seconds());
            telemetry.addData("Encoders: ", encoderdistance);
            telemetry.addData("Heading: ", heading);
            telemetry.update();
            sleep(50);

            if(Math.abs(encoderdistance - distance*RobotConstants.Tickspercm) < 1*RobotConstants.Tickspercm){ done = true;}

        }
        SetDrivePower(0);
        Gyro.calibrate();
        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void DrivePID(float speed, double distance, Telemetry telemetry, double timeout,LinearOpMode opMode) throws InterruptedException{
        PIDController encoders = new PIDController((int)(distance * RobotConstants.Tickspercm),0);
        boolean done = false;
        float EMV, encoderdistance;
        ElapsedTime time = new ElapsedTime(0);
        int donecounter = 0;


        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoders.setPIDParam(PIDController.PID_SETPOINT,(int)(distance * RobotConstants.Tickspercm));
        encoders.setPIDParam(PIDController.PID_LIMITHIGH,speed*10);
        encoders.setPIDParam(PIDController.PID_LIMITLOW,speed*-10);
        encoders.setPIDParam(PIDController.PID_KP,.02f);
       // encoders.setPIDParam(PIDController.PID_KI,.002f);
       //encoders.setPIDParam(PIDController.PID_KD,.025f);
        encoders.setPIDParam(PIDController.PID_DEADBAND,.5f);

        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(FrontLeft.getCurrentPosition() != 0){}
        time.reset();

        while(opMode.opModeIsActive() && time.seconds() < timeout && !done){
            encoderdistance = ((FrontLeft.getCurrentPosition() + FrontRight.getCurrentPosition() + BackLeft.getCurrentPosition() + BackRight.getCurrentPosition())/4);

            EMV =encoders.doPID((int)encoderdistance);
            SetDrivePower(EMV/10);

            telemetry.addData("Speed: ", EMV/10);
            telemetry.addData("time: ", time.seconds());
            telemetry.addData("Encoders: ", encoderdistance);
            telemetry.update();
            sleep(50);

            if(Math.abs(encoderdistance - distance*RobotConstants.Tickspercm) < 1*RobotConstants.Tickspercm){
                donecounter ++;
                if(donecounter > 10){
                    done = true;
                }

            }
            else{
                donecounter = 0;
            }

        }
        SetDrivePower(0);
        DriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void NoCalibratePIDTurn(Direction direction,int degrees, float MaxPowah,Telemetry telemetry,float timeout, LinearOpMode opMode ) throws InterruptedException{
        ElapsedTime time = new ElapsedTime(0);
        float heading,MV;
        PIDController PID = new PIDController(10,0);
        boolean done;
        double donecounter=0;
        if (direction == Direction.ClockWise){
            degrees = -degrees;
        }




        PID.setPIDParam(PIDController.PID_SETPOINT,degrees);
        PID.setPIDParam(PIDController.PID_LIMITHIGH,MaxPowah*10);
        PID.setPIDParam(PIDController.PID_LIMITLOW,MaxPowah * -10);
        PID.setPIDParam(PIDController.PID_KP,.3f);
        PID.setPIDParam(PIDController.PID_DEADBAND, .6f);





        MV = 1;
        time.reset();
        done = false;

        while( opMode.opModeIsActive() && time.seconds() < timeout && !done){
            heading = Gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
            MV = PID.doPID((int)heading);
            FrontRight.setPower(MV/10);
            BackRight.setPower(MV/10);
            FrontLeft.setPower(-1*MV/10);
            BackLeft.setPower(-1*MV/10);

            telemetry.addData("Speed: ", MV/10);
            telemetry.addData("Gyro Heading: ", heading);
            telemetry.addData("Time: ", time.seconds());
            telemetry.update();
            sleep(50);
            if(Math.abs((int)heading - degrees) < 2){
                donecounter++;
                if(donecounter>10){
                    done = true;
                }
            }
            else{
                donecounter = 0;
            }

        }
        SetDrivePower(0);


    }
}







