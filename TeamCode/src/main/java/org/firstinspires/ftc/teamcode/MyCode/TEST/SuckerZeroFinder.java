package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MyCode.*;
/**
 * Created by user on 1/11/18.
 */
@TeleOp
public class SuckerZeroFinder extends OpMode{

    Robot robot = new Robot();

    @Override
    public void init(){

        robot.initialize(hardwareMap,telemetry);



    }




    @Override
    public void loop(){


        float num = gamepad1.right_stick_y/2;

        telemetry.addData("Power",num);

        robot.Suckers(num);




    }
}
