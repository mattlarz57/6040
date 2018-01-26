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
public class BackRightEncoder extends LinearOpMode{


    Robot robot = new Robot();
    int counter;


    @Override
    public void runOpMode(){
        robot.initialize(hardwareMap,telemetry);
        robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        counter =1;

        waitForStart();

        if(counter == 1){

            robot.BackRight.setTargetPosition(1000);
            robot.BackLeft.setTargetPosition(1000);
            robot.FrontRight.setTargetPosition(1000);
            robot.FrontLeft.setTargetPosition(1000);
            robot.BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.BackRight.setPower(.5);
            robot.BackLeft.setPower(.5);
            robot.FrontRight.setPower(.5);
            robot.FrontLeft.setPower(.5);
            while(robot.BackRight.isBusy()||robot.BackLeft.isBusy() ||
                    robot.FrontLeft.isBusy() || robot.FrontRight.isBusy()){
                telemetry.addData("posBR", robot.BackRight.getCurrentPosition());
                telemetry.addData("PosBL", robot.BackLeft.getCurrentPosition());
                telemetry.addData("PosFR", robot.FrontRight.getCurrentPosition());
                telemetry.addData("PosFL", robot.FrontLeft.getCurrentPosition());

                telemetry.update();

            }

        }



    }




}
