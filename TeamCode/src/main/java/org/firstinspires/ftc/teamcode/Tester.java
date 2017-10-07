package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.opencv.core.Size;

/**
 * Created by user on 2017-08-29.
 */
@Autonomous
public class Tester extends LinearOpMode {

    Robot robot = new Robot();
    BasicVisionSample vision = new BasicVisionSample();
    VisionOpMode visionOpMode;


    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() throws InterruptedException{
        super.init();
        robot.initialize(hardwareMap, telemetry);
        vision.setCamera(Cameras.PRIMARY);
        vision.setFrameSize(new Size(900, 900));
       // enableExtension(VisionOpMode.Extensions.BEACON);         //Beacon detection

        vision.beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        vision.beacon.setColorToleranceBlue(0);
        vision.beacon.setColorToleranceRed(0);


        //AutoTransitioner.transitionOnStop(this,"testertele");



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);


        telemetry.addLine("Initialization Complete");
        waitForStart();
        double startruntime = getRuntime();


        //relicTrackables.activate();

        while(opModeIsActive()) {
            RelicRecoveryVuMark vumark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addLine("heading "+ Math.round(robot.getheading()));
            telemetry.addData("Time Elapsed: ", Math.round(getRuntime()-startruntime));
            telemetry.addData("Beacon Color", vision.beacon.getAnalysis().getColorString());

            telemetry.update();
        }


        //sleep(10000);
        //robot.Sideways("Right",.2,10);

    }

}

