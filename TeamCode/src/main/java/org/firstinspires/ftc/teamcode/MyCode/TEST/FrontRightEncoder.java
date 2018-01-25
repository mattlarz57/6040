package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.PushbotAutoDriveByEncoder_Linear;
import org.firstinspires.ftc.teamcode.MyCode.Robot;

/**
 * Created by user on 1/20/18.
 */
@Autonomous
public class FrontRightEncoder extends LinearOpMode{


    Robot robot = new Robot();
    int counter;


    @Override
    public void runOpMode(){
        robot.initialize(hardwareMap,telemetry);
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        counter =1;

        waitForStart();

        if(counter == 1){

            robot.FrontRight.setTargetPosition(1000);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRight.setPower(.5);
            while(robot.FrontRight.isBusy()){
                telemetry.addData("pos", robot.FrontRight.getCurrentPosition());
                telemetry.update();

            }

        }



    }




}
