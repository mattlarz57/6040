package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MyCode.Robot;

/**
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 3/20/18.
 */
public class PIDDrivetest extends LinearOpMode {

    Robot robot = new Robot();
    //PIDController PID = new PIDController(90,0);
    float heading,MV, bnoheading,counter;
    ElapsedTime time = new ElapsedTime(0);


    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addLine("Initializing");
        telemetry.update();

        robot.initialize(hardwareMap,telemetry);

       /*PID.setPIDParam(PIDController.PID_SETPOINT,90);
        PID.setPIDParam(PIDController.PID_LIMITHIGH,7.5f);
        PID.setPIDParam(PIDController.PID_LIMITLOW,-7.5f);
        PID.setPIDParam(PIDController.PID_KP,.7f);
        PID.setPIDParam(PIDController.PID_DEADBAND, 1.9f);
        */


        robot.Gyro.calibrate();
        while (robot.Gyro.isCalibrating()){}


        //MV = PID.doPID(0);

        waitForStart();
        counter =1;



        while(opModeIsActive()){

            /*//while(MV > 0) {
                sleep(50);
                heading = robot.Gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
                MV = PID.doPID((int) heading);
            /*robot.FrontRight.setPower(MV/10);
            robot.BackRight.setPower(MV/10);
            robot.FrontLeft.setPower(-MV/10);
            robot.BackLeft.setPower(-MV/10);

                telemetry.addData("Speed: ", MV / 10);
                telemetry.addData("Gyro Heading: ", heading);
                telemetry.addData("counter", counter);
                telemetry.update();
            //}
            */

            if(counter == 1){
                robot.PIDTurn(Robot.Direction.CounterClockWise,90,.75f,telemetry,30,this);
                counter ++;
            }



        }











    }







}
