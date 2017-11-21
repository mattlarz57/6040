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
    String VuMarkSeen;
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

        rotation.enableAutoRotate();
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
                if (vuMark != RelicRecoveryVuMark.UNKNOWN){
                    VuMarkSeen = vuMark.toString();
                }
                telemetry.addData("Vumark:", VuMarkSeen);
                telemetry.addLine(Jewels);
                telemetry.addLine("blue, red");
                telemetry.addLine("Right Blue?" + RightBlue);
                telemetry.addData("Time Elapsed: ", Math.round(getRuntime() - startruntime));
                telemetry.update();

                if (counter == 1) {
                    robot.Jeweler1.setPosition(RobotConstants.Jeweler1_Down);
                    sleep(100);
                    robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
                    sleep(500);
                    if (!RightBlue) {
                        robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Right);
                        sleep(250);
                        robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);

                    }
                    else if (RightBlue) {
                        robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Right);
                        sleep(250);
                        robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
                        counter = 2;

                    }

                }
                if (counter == 2){
                    robot.Jeweler1.setPosition(RobotConstants.Jeweler1_Up);
                    robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
                    counter = 3;
                }

                if(counter == 3){
                    switch(VuMarkSeen){

                        case "Center":
                            robot.Move(-1,70);

                        case "Right":
                            robot.Move(-1,90);

                        default:
                            robot.Move(-1,50);
                    }
                    counter = 4;

                }

                if(counter == 4){
                    robot.Sideways("Right",.5,15);
                    robot.DegreeTurn("CounterClockWise",.75,90);
                }


            }


        }

    }
}
