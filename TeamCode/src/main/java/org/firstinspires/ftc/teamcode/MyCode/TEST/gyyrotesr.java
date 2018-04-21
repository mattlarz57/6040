package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MyCode.Robot;

/**
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 3/09/18.
 */
public class gyyrotesr extends LinearOpMode {



    Robot robot = new Robot();



    @Override
    public void runOpMode(){

        robot.initialize(hardwareMap,telemetry);


        waitForStart();


        while(opModeIsActive()) {
            if (!robot.Gyro.isCalibrating()) {
                telemetry.addData("X: ", robot.Gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("Y: ", robot.Gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
                telemetry.addData("Z: ", robot.Gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
                telemetry.update();

            }
        }
    }

}
