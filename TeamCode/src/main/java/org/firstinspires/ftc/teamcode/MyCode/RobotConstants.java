package org.firstinspires.ftc.teamcode.MyCode;
import com.google.gson.FieldNamingPolicy;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotConstants {
    public static final float CM_Per_Inch = 2.54f;

    public static final double TicksPerRev = 28;
    public static final double MotorGearRatio =  40 * 2/3;
    public static final double WheelCircumfrence_in = Math.PI * 4.0;
    public  static final double in2cm = 2.54;
    public  static final double Tickspercm = (TicksPerRev * MotorGearRatio) / (WheelCircumfrence_in * in2cm);
    public  static final double Sidewaystickspercm = 1000000/(9*11*17*15.75*in2cm);
    public   static final double cmPerDegree = .543;

    public   static final double Suckers_In = 1;
    public   static final double Suckers_Out = -1;
    public   static final double Suckers_Stay = 0;


    public   static final double SqueezerR_Open = .09;
    public   static final double SqueezerL_Open = 1;
    public   static final double SqueezerR_Close =.24;
    public   static final double SqueezerL_Close = .83;

    public   static final double Jeweler1_Down = .95;
    public   static final double Jeweler1_Up = .02;
    public  static final double Jeweler2_Right = .15;
    public   static final double Jeweler2_Left = .95;
    public   static final double Jeweler2_Middle = .6;


    public static final double BigRelicIn = 1;
    public static final double BigRelicOut = 0;
    public   static final double Small_Relic_Close = 1;
    public   static final double Small_Relic_Open = .95;
    public static final double Small_Relic_Grab = .887;
    public static final double Small_Relic_Stand = .910;

    public static final double Camera_Jewel = .55;
    public static final double Camera_VuMark = .75;
    public static final double Camera_Forward = 0;

    public static final double RightAngle = 75;
    public static final double HalfRotation = 160;

    public static final double NewSqueezerR_Open = .35;
    public static final double NewSqueezerR_Closed_FUll = 0;
    public static final double NewSqueezerR_Closed_Ish = .07;
    public static final double NewSqueezerL_Close_Full = 1;
    public static final double NewSqueezerL_Open = .7;
    public static final double NewSqueezerL_Close_Ish = .93;

    public static final double NewSucker_Out = 1;
    public static final double NewSucker_In = -1;
    public static final double NewSucker_Stay= 0;











}
