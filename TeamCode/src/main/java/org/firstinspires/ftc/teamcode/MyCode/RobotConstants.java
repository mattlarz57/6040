package org.firstinspires.ftc.teamcode.MyCode;
import com.google.gson.FieldNamingPolicy;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotConstants {
    public static final float CM_Per_Inch = 2.54f;

    public static final DcMotor.RunMode Drive_Mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final DcMotor.RunMode Position = DcMotor.RunMode.RUN_TO_POSITION;
    public static final double TicksPerRev = 7;
    public static final double MotorGearRatio = 20.0;
    public static final double WheelCircumfrence_in = Math.PI * 4.0;
    public  static final double in2cm = 2.54;
    public  static final double Tickspercm = (TicksPerRev * MotorGearRatio) / (WheelCircumfrence_in * in2cm);
    public  static final double Sidewaystickspercm = 1000000/(9*11*17*15.75*in2cm);
    public   static final double cmPerDegree = (9.85*Math.PI)/90;


    public   static final double Suckers_In = -1;
    public   static final double Suckers_Out = 1;
    public   static final double Suckers_Stay = 0;
    public  static final double TRZero = 0;  // unsure of this
    public  static final double TLZero = -0.0244;
    public  static final double BRZero = -0.137;
    public  static final double BLZero = 0;

    public   static final double SqueezerR_Open = .055;//220/255
    public   static final double SqueezerL_Open = .95;//1
    public   static final double SqueezerR_Close =.24;//0
    public   static final double SqueezerL_Close = .8;//220/255

    public   static final double Jeweler1_Down = 1;
    public   static final double Jeweler1_Up = 0;
    public  static final double Jeweler2_Right = .1;
    public   static final double Jeweler2_Left = 1;
    public   static final double Jeweler2_Middle = .6;

    public static final double BigRelicBack_In = 236/255;
    public static final double BigRelicBack_Out = 254/255;
    public static final double BigRelicFront_In = 19/255;
    public static final double BigRelicFront_Out = 1/255;
    public   static final double Small_Relic_Close = 1;
    public   static final double Small_Relic_Open = .92;

    public static final double Camera_Jewel = .45;
    public static final double Camera_VuMark = .25;
    public static final double Camera_Forward = 1;











}
