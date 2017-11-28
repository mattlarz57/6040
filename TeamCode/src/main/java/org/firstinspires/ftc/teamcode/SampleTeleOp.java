package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class SampleTeleOp extends OpMode {
    Robot robot = new Robot();




    @Override
    public void init(){
        robot.initialize(hardwareMap,telemetry);

    }



    @Override
    public void loop(){

        
    }


}
