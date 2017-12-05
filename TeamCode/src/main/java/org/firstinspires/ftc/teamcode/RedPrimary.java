package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by user on 12/05/17.
 */
@Autonomous(group = "Final")
public class RedPrimary extends LinearOpMode {
    Robot robot = new Robot();
    int counter = -1;
    VuforiaLocalizer vuforia;
    int vumarkseen;
    Robot.team TeamColor = Robot.team.Red;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initialization: Loading...");
        robot.initialize(hardwareMap, telemetry);
        robot.SetParameters();
        robot.ResetDriveEncoders();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();

        telemetry.addLine("Initialization: Success");
        counter = 1;

        waitForStart();

        while (opModeIsActive()) {
            RelicRecoveryVuMark vumark = RelicRecoveryVuMark.from(relicTemplate);
            if(vumark == RelicRecoveryVuMark.LEFT){vumarkseen =1;}
            else if(vumark == RelicRecoveryVuMark.CENTER){vumarkseen = 2;}
            else if(vumark == RelicRecoveryVuMark.RIGHT){vumarkseen =3;}

            if (counter == 1) {
                robot.WackJewel(TeamColor);
                sleep(500);
                counter++;
            }
            if (counter == 2) {
                robot.Move(.5, 37);
                sleep(500);
                counter++;
            }
            if (counter == 3) {

                switch (vumarkseen){
                    case(1):
                        robot.Move(.5,40);
                        sleep(500);
                        counter ++;

                    case(2):
                        robot.Move(.5,60);
                        sleep(500);
                        counter ++;
                    case(3):
                        robot.Move(.5,60);
                        sleep(500);
                        counter ++;
                }
            }
            if (counter == 4){
                robot.DegreeTurn("Right",.5,270);
                sleep(500);
                counter ++;
            }


        }

    }
}
