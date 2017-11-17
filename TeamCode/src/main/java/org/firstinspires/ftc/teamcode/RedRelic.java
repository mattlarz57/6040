package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
@Autonomous
public class RedRelic extends LinearVisionOpMode {

    Robot robot = new Robot();
    VuforiaLocalizer vuforia;
    int counter;
    int repeat = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initialization: Loading...");
        waitForVisionStart();

        robot.initialize(hardwareMap, telemetry);
        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));
        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);
        enableExtension(Extensions.CAMERA_CONTROL);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);
        beacon.setColorToleranceBlue(0);
        beacon.setColorToleranceRed(0);

        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
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

            while (opModeIsActive()) {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("Vumark:", vuMark);
                telemetry.addLine(Jewels);
                telemetry.addLine("blue, red");
                telemetry.addLine("Right Blue?" + RightBlue);
                telemetry.addData("Time Elapsed: ", Math.round(getRuntime() - startruntime));
                telemetry.update();

                if (counter == 1) {
                    robot.Jeweler1.setPosition(0);
                    sleep(100);
                    robot.Jeweler2.setPosition(.5);
                    sleep(500);
                    if (!RightBlue) {
                        while (repeat < 5) {
                            robot.Jeweler2.setPosition(0);
                            sleep(250);
                            robot.Jeweler2.setPosition(.5);
                            repeat ++;
                        }

                        robot.Jeweler1.setPosition(.85);
                        robot.Jeweler2.setPosition(.5);

                    }
                    else if (RightBlue) {
                        while (repeat < 5) {
                            robot.Jeweler2.setPosition(0);
                            sleep(250);
                            robot.Jeweler2.setPosition(.5);
                            repeat ++;
                        }
                        robot.Jeweler1.setPosition(.9);
                        robot.Jeweler2.setPosition(.5);
                    }
                    else {
                        telemetry.addLine("HUH?");
                    }
                }


            }


        }

    }
}
