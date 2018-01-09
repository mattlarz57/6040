package org.firstinspires.ftc.teamcode.MyCode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotConstants {
    public static final float CM_Per_Inch = 2.54f;

    public static final DcMotor.RunMode Drive_Mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final DcMotor.RunMode Position = DcMotor.RunMode.RUN_TO_POSITION;
    public static final double TicksPerRev = 28;
    public static final double MotorGearRatio = 20.0;
    public static final double WheelCircumfrence_in = Math.PI * 4.0;
    public  static final double in2cm = 2.54;
    public  static final double Tickspercm = (TicksPerRev * MotorGearRatio) / (WheelCircumfrence_in * in2cm);
    public  static final double Sidewaystickspercm = 1000000/(9*11*17*15.75*in2cm);
    public   static final double cmPerDegree = (9.85*Math.PI)/90;


    public   static final double Suckers_In = 180;
    public   static final double Suckers_Out = 0;
    public   static final double Suckers_Stay = 90;

    public   static final double SqueezerR_Open = 0;
    public   static final double SqueezerL_Open = 2 ;
    public   static final double SqueezerR_Close = 178 ;
    public   static final double SqueezerL_Close = 180;

    public   static final double Jeweler1_Down = .01;
    public   static final double Jeweler1_Up = .045;
    public  static final double Jeweler2_Right = 1;
    public   static final double Jeweler2_Left = 0;
    public   static final double Jeweler2_Middle = .55;

    public  static final double Big_Relic_Up = 0;
    public   static final double Big_Relic_Down = 1;
    public   static final double Small_Relic_Close = 1;
    public   static final double Small_Relic_Open = 0;











}
