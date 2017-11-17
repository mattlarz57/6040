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
 * Created by user on 2017-10-10.
 */
@Autonomous
public class VisionTester extends LinearVisionOpMode {

    Robot robot = new Robot();

    VuforiaLocalizer vuforia;
    int counter = 1;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addLine("Initializaton: Loading...");
        waitForVisionStart();

        robot.initialize(hardwareMap,telemetry);
        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900,900));
        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);
        enableExtension(Extensions.CAMERA_CONTROL);

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        beacon.setColorToleranceBlue(0);
        beacon.setColorToleranceRed(0);

        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        telemetry.addLine("Initialization: Success");

        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

       */

        waitForStart();

        double startruntime = getRuntime();
        String Jewels;


        while(!(beacon.getAnalysis().isBeaconFound())) {
            telemetry.addLine("heading " + Math.round(robot.getheading()));
            telemetry.addData("Time Elapsed: ", Math.round(getRuntime() - startruntime));
            telemetry.addLine("Beacon: Searching");
            telemetry.update();
        }
        Jewels = beacon.getAnalysis().getColorString();
        telemetry.addLine("Beacon: Found");
        telemetry.update();

        if(Jewels != null) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);

            relicTrackables.activate();



            while (opModeIsActive()) {

                RelicRecoveryVuMark vumark = RelicRecoveryVuMark.from(relicTemplate);

                telemetry.addData("Vumark:", vumark);
                telemetry.addLine("Jewel Orientation:" + Jewels);
                telemetry.addLine("heading " + Math.round(robot.getheading()));
                telemetry.addData("Time Elapsed: ", Math.round(getRuntime() - startruntime));
                telemetry.update();

                if (counter == 1){
                    if (Jewels == "blue,red"){

                    }
                }


               /* if(counter == 1){
                    robot.Move(.5,25);
                    counter = 2;
                }
                if(counter == 2){
                    robot.Sideways("Right",.25,10);
                    counter = 3;

                }
                */

            }



        }









    }
}
