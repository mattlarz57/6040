package org.firstinspires.ftc.teamcode.MyCode.BLUE;

import com.disnodeteam.dogecv.CameraViewDisplay;
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
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 3/15/18.
 */
@Autonomous
public class BluePrimaryCVMultiGyro extends LinearOpMode {

    Robot robot = new Robot();
    JewelDetector jewelDetector = new JewelDetector();
    //GlyphDetector glyphDetector = new GlyphDetector();
    Robot.team TeamColor = Robot.team.Red;
    ClosableVuforiaLocalizer vuforia;
    RelicRecoveryVuMark VuMarkOutput = RelicRecoveryVuMark.UNKNOWN;
    JewelDetector.JewelOrder JewelOutput = JewelDetector.JewelOrder.UNKNOWN;
    ElapsedTime elapsedTime;


    int counter;
    boolean closed = false;
    boolean disabled = false;
    boolean first = true;
    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;
    boolean NeedTime = true;
    double distance, diff, secondcm;


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

        while (robot.Gyro.isCalibrating()) {
        }
        telemetry.addLine("Initialization: Success");


        waitForStart();

        robot.ResetDriveEncoders();

        jewelDetector.enable();
        while (opModeIsActive()) {
            telemetry.update();
            telemetry.addData("Step", counter);
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (VuMarkOutput != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark: ", VuMarkOutput);
            }
            if (JewelOutput != JewelDetector.JewelOrder.UNKNOWN) {
                telemetry.addData("Jewel Order: ", JewelOutput);
            }
            telemetry.addData("Jewel Detector Closed?: ", disabled);
            telemetry.addData("Vuforia Closed?: ", closed);
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
                robot.WackJewel(TeamColor, JewelOutput);
                counter++;
            }
            if (counter == 2) {
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
            if (counter == 3) {
                ElapsedTime VuTime = new ElapsedTime();
                telemetry.addLine("Searching for VuMark");
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    VuMarkOutput = vuMark;
                    vuforia.close();
                    closed = true;
                    robot.Camera.setPosition(RobotConstants.Camera_Forward);

                    if (first) {
                        counter = 4;
                    } else {
                        counter = 99;
                    }
                } else if (VuTime.seconds() >= 3.5) {
                    counter = 98;
                }
            }
            if (counter == 4) {
                if (VuMarkOutput == RelicRecoveryVuMark.RIGHT) {
                    robot.Drive(.75, 120, telemetry, 4);
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.CENTER) {
                    robot.Drive(.75, 100, telemetry, 4);
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.LEFT) {
                    robot.Drive(.75, 80, telemetry, 4);
                    counter = 5;
                }
            }
            if (counter == 5) {
                robot.GyroTurn(Robot.Direction.CounterClockWise, RobotConstants.RightAngle, .5, telemetry, 4);
                counter++;

            }
            if (counter == 6) {
                robot.Drive(.5, 17, telemetry, 3);
                counter++;
            }
            if (counter == 7) {
                robot.Suckers(RobotConstants.Suckers_Out);
                sleep(2000);
                robot.Suckers(RobotConstants.Suckers_Stay);
                robot.Drive(.75, -30, telemetry, 2);
                counter++;
            }
            if (counter == 8) {
                robot.GyroTurn(Robot.Direction.ClockWise, RobotConstants.HalfRotation, .5, telemetry, 3);
                counter++;
            }
            if (counter == 9) {
                robot.Suckers(RobotConstants.Suckers_In);
                distance = robot.DriveWithSuck(.75, 95, telemetry, 4.5);
                counter++;
            }
            if (counter == 10) {
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Close);
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Close);
                robot.Drive(.75, -(distance / RobotConstants.Tickspercm), telemetry, 4.5);
                counter++;
            }
            if (counter == 11) {
                robot.Glyphter.setPower(-.6);
                robot.GyroTurn(Robot.Direction.CounterClockWise, RobotConstants.HalfRotation, .5, telemetry, 3);
                robot.Glyphter.setPower(0);
                counter++;

            }
            if (counter == 12) {
                robot.Drive(.5, 35, telemetry, 3);
                counter++;
            }
            if (counter == 13) {
                robot.Suckers(RobotConstants.Suckers_Out);
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Open);
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Open);
                sleep(2000);
                robot.Suckers(RobotConstants.Suckers_Stay);
                robot.Drive(.75, -17, telemetry, 2);
                counter++;
            }
            if (counter == 14) {

            }


            if (counter == 98) {
                if (first) {
                    robot.Drive(.5, 20, telemetry, 2);
                    robot.Camera.setPosition(RobotConstants.Camera_Jewel);
                    first = false;
                    counter = 3;
                } else {
                    VuMarkOutput = RelicRecoveryVuMark.RIGHT;
                    counter = 99;
                }
            }
            if (counter == 99) {
                if (VuMarkOutput == RelicRecoveryVuMark.RIGHT) {
                    robot.Drive(.75, 100, telemetry, 3);
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.CENTER) {
                    robot.Drive(.75, 80, telemetry, 4);
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.LEFT) {
                    robot.Drive(.75, 60, telemetry, 4);
                    counter = 5;
                }
            }


        }

    }
}
