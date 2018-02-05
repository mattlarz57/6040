package org.firstinspires.ftc.teamcode.MyCode.BLUE;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;
import org.firstinspires.ftc.teamcode.OtherFiles.ClosableVuforiaLocalizer;

/**
 * Created by user on 2/03/18.
 */
@Autonomous
public class BluePrimaryCV extends LinearOpMode {
    Robot robot = new Robot();
    JewelDetector jewelDetector = new JewelDetector();
    GlyphDetector glyphDetector = new GlyphDetector();
    Robot.team TeamColor = Robot.team.Blue;
    ClosableVuforiaLocalizer vuforia;
    RelicRecoveryVuMark VuMarkOutput = RelicRecoveryVuMark.UNKNOWN;
    JewelDetector.JewelOrder JewelOutput = JewelDetector.JewelOrder.UNKNOWN;



    int counter;
    boolean closed = false;
    boolean disabled = false;
    boolean first = true;
    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;
    boolean NeedTime = true;
    double Vutime, Glyphtime;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initialization: Loading...");
        robot.initialize(hardwareMap, telemetry);
        robot.Camera.setPosition(RobotConstants.Camera_Jewel);
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;


        telemetry.update();
        telemetry.addLine("Initialization: Success");


        waitForStart();
        ElapsedTime elapsedTime = new ElapsedTime(0);
        jewelDetector.enable();
        robot.ResetDriveEncoders();
        while (opModeIsActive()) {
            if (counter == 8 || getRuntime() > 30) {
                robot.Suckers(RobotConstants.Suckers_Stay);
                robot.SetDrivePower(0);
                requestOpModeStop();
            }
            telemetry.addData("Step", counter);
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (VuMarkOutput != RelicRecoveryVuMark.UNKNOWN || JewelOutput != JewelDetector.JewelOrder.UNKNOWN) {
                telemetry.addData("VuMark: ", VuMarkOutput);
                telemetry.addData("Jewel Order", JewelOutput);
            }
            telemetry.addData("Jewel Closed?: ", disabled);
            telemetry.addData("Vuforia Closed?:  ", closed);
            telemetry.update();
            if (!disabled) {
                telemetry.addLine("Searching For Jewel");
                if (jewelDetector.getLastOrder() != JewelDetector.JewelOrder.UNKNOWN) {
                    JewelOutput = jewelDetector.getLastOrder();
                    jewelDetector.disable();
                    disabled = true;
                    counter = 1;
                }
            }
            if (counter == 1) {
                try {
                    robot.WackJewel(TeamColor, JewelOutput);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                counter++;
            }
            if (disabled && counter == 2) {
                robot.Camera.setPosition(RobotConstants.Camera_VuMark);
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                parameters.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                vuforia = new ClosableVuforiaLocalizer(parameters);
                relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                relicTemplate = relicTrackables.get(0);
                relicTrackables.activate();
                counter = 3;
            }
            if (counter == 3 && NeedTime) {
                Vutime = getRuntime();
                NeedTime = false;
            } else if (counter == 3) {
                telemetry.addLine("Searching for VuMark");
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    VuMarkOutput = vuMark;
                    vuforia.close();
                    closed = true;
                    robot.Camera.setPosition(RobotConstants.Camera_Forward);
                    counter = 4;
                } else if (getRuntime() - Vutime >= 5) {
                    vuforia.close();
                    VuMarkOutput = RelicRecoveryVuMark.RIGHT;
                    counter = 4;
                }

            }

            if (counter == 4) {
                if (VuMarkOutput == RelicRecoveryVuMark.RIGHT) {
                    robot.Drive(.35, 85, telemetry, elapsedTime,4);
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.CENTER) {
                    robot.Drive(.35, 65, telemetry,elapsedTime, 4);
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.LEFT) {
                    robot.Drive(.35, 45, telemetry, elapsedTime,4);
                    counter = 5;
                }

            }

            if (counter == 5) {
                robot.EncoderTurn(Robot.Direction.CounterClockWise, .5, 50, elapsedTime,3);
                counter++;
            }
            if (counter == 6) {
                robot.Drive(.35, 23, telemetry,elapsedTime, 3);
                counter++;

            }
            if (counter == 7) {
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Open);
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Open);
                robot.Suckers(RobotConstants.Suckers_Out);
                sleep(2000);
                robot.Suckers(RobotConstants.Suckers_Stay);
                robot.Drive(.35, -25, telemetry, elapsedTime,3);
                counter++;
            }

            /*
            if (counter == 8) {
                robot.EncoderTurn(Robot.Direction.ClockWise, .5, 215);
                NeedTime = true;
                counter++;

            }
            if (counter == 9) {
                robot.Drive(.35, 100, telemetry);
                counter++;
            }
            if (counter == 10) {
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Close);
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Close);
                robot.Suckers(RobotConstants.Suckers_In);
                sleep(2000);

                robot.Drive(.35, -100, telemetry);
                robot.Suckers(RobotConstants.Suckers_Stay);
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Open);
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Open);

                counter++;
            }
            if (counter == 11) {
                robot.EncoderTurn(Robot.Direction.CounterClockWise, .5, 215);
                counter++;
            }
            if (counter == 12) {
                robot.Drive(.35, 27, telemetry);
                counter++;
            }
            if (counter == 13) {
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Open);
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Open);
                robot.Suckers(RobotConstants.Suckers_Out);
                sleep(2000);
                robot.Suckers(RobotConstants.Suckers_Stay);
                robot.Drive(.35, -27, telemetry);
                counter++;

            }
            */

        }

    }
}
