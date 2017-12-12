package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Vision.LinearVisionOpMode;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by user on 2017-11-04.
 */

public class RedRelic extends LinearVisionOpMode {

    Robot robot = new Robot();
    VuforiaLocalizer vuforia;
    int counter = -1;
    int repeat = 0;
    int vuMarkSeen = 0;


    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initialization: Loading...");
        waitForVisionStart();


        robot.initialize(hardwareMap, telemetry);
        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));
        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);
        enableExtension(Extensions.CAMERA_CONTROL);

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.DEFAULT);
        beacon.setColorToleranceBlue(0);
        beacon.setColorToleranceRed(0);

        rotation.enableAutoRotate();
        robot.ResetDriveEncoders();
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Initialization: Success");

        waitForStart();


        double startruntime = getRuntime();
        String Jewels;




        while (!(beacon.getAnalysis().isBeaconFound() || getRuntime() - startruntime > 5)) {
            telemetry.addData("Time Elapsed: ", Math.round(getRuntime() - startruntime));
            telemetry.addLine("Beacon: Searching");
            telemetry.update();

        }
        Jewels = beacon.getAnalysis().getColorString();
        boolean RightBlue = beacon.getAnalysis().isRightBlue();
        telemetry.addLine("Beacon: Found");
        telemetry.addLine("Right Blue?" + RightBlue);
        telemetry.update();

        if ( Jewels != null || (getRuntime() - startruntime) > 5) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);

            relicTrackables.activate();
            counter = 1;
            telemetry.addData("AveragePos: ",(robot.FrontLeft.getCurrentPosition() + robot.FrontRight.getCurrentPosition()+robot.BackRight.getCurrentPosition()+robot.BackLeft.getCurrentPosition())/4);
            telemetry.addData("VuMark: 1=R,2=C,3=L ",vuMarkSeen);
            telemetry.addData("Step: ", counter);
            telemetry.addLine("Right Blue? " + RightBlue);


            while (opModeIsActive()) {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if(vuMark == RelicRecoveryVuMark.RIGHT){ vuMarkSeen = 1;}
                else if (vuMark == RelicRecoveryVuMark.CENTER){ vuMarkSeen = 2;}
                else if (vuMark == RelicRecoveryVuMark.LEFT){ vuMarkSeen = 3;}


                if (counter == 1) {
                    telemetry.addLine("Right Blue? " + RightBlue);
                    telemetry.addData("Step: ", counter);
                    telemetry.update();
                    robot.Jeweler1.setPosition(RobotConstants.Jeweler1_Down);
                    sleep(150);
                    robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
                    sleep(500);
                    if (RightBlue) {
                        telemetry.addLine("Right Blue? " + RightBlue);
                        telemetry.addData("Step: ", counter);
                        telemetry.update();
                        robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Left);
                        sleep(250);
                        robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
                        telemetry.update();
                        counter = 2;

                    }
                    else if (!RightBlue) {
                        telemetry.addLine("Right Blue? " + RightBlue);
                        telemetry.addData("Step: ", counter);
                        telemetry.update();
                        robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Right);
                        sleep(250);
                        robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
                        telemetry.update();
                        counter = 2;

                    }

                }
                if (counter == 2){
                    telemetry.addData("AveragePos: ",(robot.FrontLeft.getCurrentPosition() + robot.FrontRight.getCurrentPosition()+robot.BackRight.getCurrentPosition()+robot.BackLeft.getCurrentPosition())/4);
                    telemetry.addData("VuMark: 1=R,2=C,3=L ",vuMarkSeen);
                    telemetry.addData("Step: ", counter);
                    telemetry.update();
                    robot.Jeweler1.setPosition(RobotConstants.Jeweler1_Up);
                    robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
                    sleep(2500);
                    telemetry.addData("AveragePos: ",(robot.FrontLeft.getCurrentPosition() + robot.FrontRight.getCurrentPosition()+robot.BackRight.getCurrentPosition()+robot.BackLeft.getCurrentPosition())/4);
                    telemetry.addData("VuMark: 1=R,2=C,3=L ",vuMarkSeen);
                    telemetry.addData("Step: ", counter);
                    telemetry.update();
                    counter = 999;
                }
                if (counter == 3){
                    telemetry.addData("AveragePos: ",(robot.FrontLeft.getCurrentPosition()   ));//+ robot.FrontRight.getCurrentPosition()+robot.BackRight.getCurrentPosition()+robot.BackLeft.getCurrentPosition())/4);
                    telemetry.addData("VuMark: 1=R,2=C,3=L ",vuMarkSeen);
                    telemetry.addData("Step: ", counter);
                    telemetry.update();
                    robot.Backwards(.01,-.1);
                    sleep(1000);
                    telemetry.addData("AveragePos: ",(robot.FrontLeft.getCurrentPosition()   )); //+ robot.FrontRight.getCurrentPosition()+robot.BackRight.getCurrentPosition()+robot.BackLeft.getCurrentPosition())/4);
                    telemetry.addData("VuMark: 1=R,2=C,3=L ",vuMarkSeen);
                    telemetry.addData("Step: ", counter);
                    telemetry.update();

                    counter = 9;
                }

                if(counter == 4){
                    telemetry.addData("AveragePos: ",(robot.FrontLeft.getCurrentPosition() + robot.FrontRight.getCurrentPosition()+robot.BackRight.getCurrentPosition()+robot.BackLeft.getCurrentPosition())/4);
                    telemetry.addData("VuMark: 1=R,2=C,3=L ",vuMarkSeen);
                    telemetry.addData("Step: ", counter);
                    telemetry.update();
                    switch(vuMarkSeen){

                        case (2):
                            robot.Backwards(.1,-50);

                        case (3):
                            robot.Backwards(.1,-70);

                        default:
                            robot.Backwards(.1,-30);
                    }
                    telemetry.update();

                    counter = 5;

                }

                if(counter == 5){
                    telemetry.addData("AveragePos: ",(robot.FrontLeft.getCurrentPosition() + robot.FrontRight.getCurrentPosition()+robot.BackRight.getCurrentPosition()+robot.BackLeft.getCurrentPosition())/4);
                    telemetry.addData("VuMark: 1=R,2=C,3=L ",vuMarkSeen);
                    telemetry.addData("Step: ", counter);
                    telemetry.update();
                    robot.Sideways("Right",.5,15);
                }


            }


        }

    }
}
