package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by user on 12/04/17.
 */
public class EncoderDrivetest extends LinearOpMode {

    DcMotor FR,FL,BR,BL;
    int counter =1;


    public void runOpMode() throws InterruptedException{

        BR = hardwareMap.dcMotor.get("2a");
        BL = hardwareMap.dcMotor.get("2b");
        FR = hardwareMap.dcMotor.get("1a");
        FL = hardwareMap.dcMotor.get("1b");



        waitForStart();

        while(opModeIsActive()) {
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            if (counter == 1) {
                double ticks = 10 * RobotConstants.Tickspercm;
                double BRPos = Math.abs(BR.getCurrentPosition());
                double BLPos = Math.abs(BL.getCurrentPosition());
                double FRPos = Math.abs(FR.getCurrentPosition());
                double FLPos = Math.abs(FL.getCurrentPosition());

                while (BRPos<ticks && BLPos<ticks && FRPos<ticks && FLPos<ticks) {
                    FR.setPower(.25);
                    BR.setPower(.25);
                    FL.setPower(.25);
                    BL.setPower(.25);
                    BRPos = Math.abs(BR.getCurrentPosition());
                    BLPos = Math.abs(BL.getCurrentPosition());
                    FRPos = Math.abs(FR.getCurrentPosition());
                    FLPos = Math.abs(FL.getCurrentPosition());
                    telemetry.addData("FR",FRPos);
                    telemetry.addData("FL",FLPos);
                    telemetry.addData("BR",BRPos);
                    telemetry.addData("BL",BLPos);
                }
                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);
                counter = 2;


            }
        }
    }
}



