package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

class RobotConstants {
    public static final float CM_Per_Inch = 2.54f;

    static final DcMotor.RunMode Drive_Mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final DcMotor.RunMode Position = DcMotor.RunMode.RUN_TO_POSITION;
    static final double TicksPerRev = 28;
    static final double MotorGearRatio = 20.0;
    static final double WheelCircumfrence_cm = Math.PI * 4.0;
    static final double in2cm = 2.54;
    static final double Tickspercm = (TicksPerRev * MotorGearRatio) / (WheelCircumfrence_cm *in2cm );
    static final double Sidewaystickspercm = 1000000/(9*11*17*15.75*in2cm);

    static final double SqueezerR_Open = .05;
    static final double SqueezerL_Open = .95;
    static final double SqueezerR_Close = .21;
    static final double SqueezerL_Close = .79;



    static final double Jeweler1_Down = .15;
    static final double Jeweler1_Up = .97;
    static final double Jeweler2_Right = 1;
    static final double Jeweler2_Left = 0;
    static final double Jeweler2_Middle = .5;





    static final int Glyphter_TicksPerRev = 28;
    static final int Glyphter_GearRatio = 30;
    static final float Glyphter_Circumfrence = 3.14f*2f;
    static final float Glyphter_TicksPerInch = Glyphter_TicksPerRev*Glyphter_GearRatio/Glyphter_Circumfrence;
    static final int Glyphter_2Up_Counts = Math.round(14*Glyphter_TicksPerInch);


    static final double Glyphter_Suck = 1;
    static final double Glyphter_Spit = -1;
    static final double Glypther_Stay = 0;








}
