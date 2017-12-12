package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
    int counter;
    VuforiaLocalizer vuforia;
    int vumarkseen;
    Robot.team TeamColor = Robot.team.Red;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initialization: Loading...");
        robot.initialize(hardwareMap, telemetry);
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
            if(vumark == RelicRecoveryVuMark.LEFT){vumarkseen = 1;}
            else if(vumark == RelicRecoveryVuMark.CENTER){vumarkseen = 2;}
            else if(vumark == RelicRecoveryVuMark.RIGHT){vumarkseen = 3;}

            if (counter == 1) {
                robot.WackJewel(TeamColor);
                counter++;
            }
            if (counter == 2) {
                robot.Move(-.5, 10);
                counter++;
            }
            if (counter == 3) {

                if (vumarkseen == 1) {
                    robot.Move(-.5,25);
                    sleep(300);
                    counter = 4;
                }
                else if (vumarkseen == 2){
                    robot.Move(-.5,45);
                    sleep(300);
                    counter = 4;
                }
                else if (vumarkseen == 3){

                    robot.Move(-.5,60);
                    sleep(300);
                    counter = 4;
                }

            }

            if (counter == 4){
              robot.EncoderTurn("CounterClockWise",.5,90);
                sleep(500);
                counter ++;
            }
            if(counter == 5){
                robot.Move(.5,10);
                counter ++;
            }
            if(counter == 6){
                robot.Suckers(RobotConstants.Suckers_Out);
                sleep(250);
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Open);
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Open);
                counter ++;
            }
            if( counter == 7){
                robot.Move(-.5,20);
                robot.Suckers(RobotConstants.Suckers_Stay);
                counter ++;
            }
            if(counter == 8){
                robot.EncoderTurn("ClockWise",1,180);
                sleep(1500);
                counter ++;
            }
            if(counter == 9){
                robot.Suckers(RobotConstants.Suckers_In);
                counter ++;
            }
            if(counter == 10){
                robot.Move(1,100);
                counter ++;
            }
            if(counter == 11){
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Close);
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Close);
                counter ++;
            }
            if (counter == 12) {
                robot.Move(-1,100);
                counter ++;
            }
            if(counter == 13){
                robot.EncoderTurn("CounterClockWise",1,180);
                sleep(1500);
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Open);
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Open);
                counter ++;
            }
            if(counter == 14){
                robot.Move(1,20);
                counter ++;
            }
            if(counter == 15){
                robot.Suckers(RobotConstants.Suckers_Out);
                counter ++;
            }
            if(counter == 16){
                robot.Move(-1,10);
            }

        }

    }
}
