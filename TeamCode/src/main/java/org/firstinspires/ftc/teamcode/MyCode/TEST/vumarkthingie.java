package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class vumarkthingie extends LinearOpMode{

        VuforiaLocalizer vuforia;

        @Override public void runOpMode() {


            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate");

            waitForStart();

            relicTrackables.activate();

            while (opModeIsActive()) {

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    telemetry.addData("VuMark", "%s visible", vuMark);
                }
                else {
                    telemetry.addData("VuMark", "not visible");
                }
                telemetry.update();
            }
        }
    }


