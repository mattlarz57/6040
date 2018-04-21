
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



package org.firstinspires.ftc.teamcode.MyCode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MyCode.Robot;


@Autonomous(group = "NEW")
public class NewHUh extends LinearOpMode {


    Robot robot = new Robot();
    @Override public void runOpMode() {

        // get a reference to our compass

        int counter;
        robot.initnew(hardwareMap);

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Back cm: ",robot.BackSide.getDistance(DistanceUnit.CM));
            telemetry.addData("Front cm: ", robot.Side.getDistance(DistanceUnit.CM));
            telemetry.update();
         /* if (rangeSensor.getDistance(DistanceUnit.CM) > 38)
            {
                BackRight.setPower(.5);
                FrontRight.setPower(-.5);
                BackLeft.setPower(.5);
                FrontLeft.setPower(-.5);
            }

           else{
                BackRight.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                FrontLeft.setPower(0);        }
        }*/

        }

    }

}









/*package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.disnodeteam.dogecv.math.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.MyCode.Robot;



@Autonomous
public class HUH extends LinearOpMode {
    ElapsedTime elapsedTime;
    Robot robot = new Robot();
    ;

    @Override
    public void runOpMode() throws InterruptedException {
        elapsedTime = new ElapsedTime(0);


        waitForStart();


        while (opModeIsActive()) {

            robot.Rangemovement(.5);

            telemetry.addData("time", elapsedTime.seconds());
            telemetry.update();





        }

    }
}*/
