package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class SampleTeleOp extends OpMode {

    DcMotor Starwars1, Starwars2;
    Servo Cringe;




    @Override
    public void init(){
        Cringe = hardwareMap.servo.get("Servo");
        Starwars1 = hardwareMap.dcMotor.get("Motor1");
        Starwars2 = hardwareMap.dcMotor.get("Motor2");
        Starwars1.setDirection(DcMotorSimple.Direction.REVERSE);

        Cringe.setPosition(35);
    }



    @Override
    public void loop(){

        double RightPower = gamepad1.right_stick_y;
        double LeftPower = gamepad1.left_stick_y;
        Starwars1.setPower(RightPower);
        Starwars2.setPower(LeftPower);


        if(gamepad1.right_trigger > .1 ){
            Cringe.setPosition(200);
        }
        else if(gamepad1.left_trigger>.1){
            Cringe.setPosition(35);
        }


    }


}
