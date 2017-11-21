package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
    RobotConstants robotConstants = new RobotConstants();




    public DcMotor BackRight, BackLeft, FrontRight, FrontLeft, Glyphter, relicArm;
    public Servo Jeweler1, Jeweler2, SqueezerR, SqueezerL, relicBig,relicSmall;
    public CRServo GBR,GBL,GTR,GTL;
    public BNO055IMU bno055IMU;
    public TouchSensor Touch;



    Telemetry t;


    public boolean initialize(HardwareMap hardwareMap, Telemetry telemetry) {

        t = telemetry;

        BackRight=hardwareMap.dcMotor.get("2a");
        BackLeft= hardwareMap.dcMotor.get("2b");
        FrontRight = hardwareMap.dcMotor.get("1a");
        FrontLeft = hardwareMap.dcMotor.get("1b");
        Glyphter = hardwareMap.dcMotor.get("3a");
        relicArm = hardwareMap.dcMotor.get("3b");
        GTL = hardwareMap.crservo.get("3");
        GBL = hardwareMap.crservo.get("4");
        GTR = hardwareMap.crservo.get("5");
        GBR = hardwareMap.crservo.get("6");
        Jeweler1 = hardwareMap.servo.get("1");
        Jeweler2 = hardwareMap.servo.get("2");
        SqueezerL = hardwareMap.servo.get("B1");
        SqueezerR = hardwareMap.servo.get("B2");
        relicBig = hardwareMap.servo.get("B3");
        relicSmall = hardwareMap.servo.get("B4");
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        GTR.setDirection(DcMotorSimple.Direction.REVERSE);
        GBL.setDirection(DcMotorSimple.Direction.REVERSE);
        bno055IMU = hardwareMap.get(BNO055IMU.class,"IMU");
        Touch = hardwareMap.touchSensor.get("Touch");
        Jeweler2.setPosition(robotConstants.Jeweler2_Left);
        Jeweler1.setPosition(RobotConstants.Jeweler1_Up);
        SqueezerL.setPosition(.79);
        SqueezerR.setPosition(.21);
        relicBig.setPosition(.15);
        GTR.setPower(0);
        GTL.setPower(0);
        GBR.setPower(0);
        GBL.setPower(0);


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

    public void SetParameters(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.i2cAddr = BNO055IMU.I2CADDR_DEFAULT;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        bno055IMU.initialize(parameters);

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
    }


   /* public void GyroTurn(String direction, double power, double degrees, Telemetry telemetry) {
       telemetry.addData("Heading", robot.getheading());
       if (direction == "ClockWise") {
           while (robot.getheading() < degrees - 3) {
               BackRight.setPower(-power);
               BackLeft.setPower(power);
               FrontRight.setPower(-power);
               FrontLeft.setPower(power);
               telemetry.addData("Heading", robot.getheading());

           }
           SetDrivePower(0);
           Gyro.calibrate();
       } else if (direction == "CounterClockWise") {

           while (robot.getheading() > degrees + 3) {
               BackRight.setPower(power);
               BackLeft.setPower(-power);
               FrontRight.setPower(power);
               FrontLeft.setPower(-power);
               telemetry.addData("Heading", robot.getheading());

           }
           SetDrivePower(0);
           Gyro.calibrate();
       }


   }
   */

    public double[] getorientaion() {
        Orientation degrees;
        degrees = bno055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        t.update();
        return new double[]{degrees.firstAngle,degrees.secondAngle,degrees.thirdAngle,0};
    }

    public float getheading() {
        Orientation degrees;
        degrees = bno055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        return (degrees.thirdAngle);
    }

    public void DegreeTurn(String Direction , double power, double degrees){
        if (Direction == "Left"){
            LeftTurn(power, degrees);
        }
        else if(Direction == "Right"){
            RightTurn(power,degrees);
        }
    }

    private void LeftTurn(double power, double degrees){
        while(getheading() > degrees-3){
            BackRight.setPower(power);
            BackLeft.setPower(power);
            FrontRight.setPower(-power);
            FrontLeft.setPower(-power);
        }
        SetDrivePower(0);

    }


    private void RightTurn(double power, double degrees){
        while(getheading() <  degrees +3){
            BackRight.setPower(-power);
            BackLeft.setPower(-power);
            FrontRight.setPower(power);
            FrontLeft.setPower(power);
        }
        SetDrivePower(0);
    }



    }







