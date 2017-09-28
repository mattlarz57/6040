package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

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


    public final double TicksPerRev = 28;
    public final double MotorGearRatio = 10.0;
    public final double WheelCircumfrence_cm = Math.PI * 4.0;
    public final double in2cm = 2.54;
    public final double Tickspercm = (TicksPerRev * MotorGearRatio) / (WheelCircumfrence_cm *in2cm );
    public final double Sidewaystickspercm = 50000 / 3003;

    public static final String BackRightName = "2a";
    public static final String BackLeftName = "2b";
    public static final String FrontRightName = "1a";
    public static final String FrontLeftName = "1b";
    public static final String IMUNAME = "IMU";


    public DcMotor BackRight, BackLeft, FrontRight, FrontLeft;
    public BNO055IMU bno055IMU;
    public DeviceInterfaceModule dim;
    public VuforiaLocalizer vuforia;



    Telemetry t;


    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {

        t = telemetry;

        BackRight = hardwareMap.dcMotor.get(BackRightName);
        BackLeft = hardwareMap.dcMotor.get(BackLeftName);
        FrontLeft = hardwareMap.dcMotor.get(FrontLeftName);
        FrontRight = hardwareMap.dcMotor.get(FrontRightName);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bno055IMU = hardwareMap.get(BNO055IMU.class,IMUNAME);
        SetParameters();


    }

    public VuforiaTrackable vuforia(HardwareMap hardwareMap, Telemetry telemetry){
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

    public void SetParameters(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.i2cAddr = BNO055IMU.I2CADDR_DEFAULT;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        bno055IMU.initialize(parameters);

    }

    public RelicRecoveryVuMark getvuMark(VuforiaTrackable relicTemplate){

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
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
        double ticks = centimeters * Tickspercm;
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
        double ticks = centimeters * Sidewaystickspercm;
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
       double heading = Gyro.getHeading();
       telemetry.addData("Heading", heading);


       if (direction == "ClockWise") {
           while (heading < degrees - 3) {
               BackRight.setPower(-power);
               BackLeft.setPower(power);
               FrontRight.setPower(-power);
               FrontLeft.setPower(power);
               heading = Gyro.getHeading();
               telemetry.addData("Heading", heading);

           }
           SetDrivePower(0);
           Gyro.calibrate();
       } else if (direction == "CounterClockWise") {

           while (heading > degrees + 3) {
               BackRight.setPower(power);
               BackLeft.setPower(-power);
               FrontRight.setPower(power);
               FrontLeft.setPower(-power);
               heading = Gyro.getHeading();
               telemetry.addData("Heading", heading);

           }
           SetDrivePower(0);
           Gyro.calibrate();
       }


   }
   */

    public double[] getorientaion() {
        Orientation degrees;
        degrees = bno055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

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
        while(getheading() > degrees){
            BackRight.setPower(power);
            BackLeft.setPower(power);
            FrontRight.setPower(-power);
            FrontLeft.setPower(-power);
        }
        SetDrivePower(0);

    }


    private void RightTurn(double power, double degrees){
        float heading;
        heading = getheading();
        while(getheading() <  degrees){
            BackRight.setPower(-power);
            BackLeft.setPower(-power);
            FrontRight.setPower(power);
            FrontLeft.setPower(power);
        }
        SetDrivePower(0);
    }



}

