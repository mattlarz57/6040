package org.firstinspires.ftc.teamcode.MyCode.BLUE.NEW;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.disnodeteam.dogecv.math.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;
import org.firstinspires.ftc.teamcode.OtherFiles.ClosableVuforiaLocalizer;

/**
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 4/11/18.
 */
@Autonomous(group = "NEWB")
public class NewBluePrimary extends LinearOpMode{
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
        robot.initnew(hardwareMap);
        robot.Camera.setPosition(RobotConstants.Camera_Jewel);
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;
        jewelDetector.rotateMat=true;

        telemetry.update();
        telemetry.addLine("Initialization: Success");


        waitForStart();
        jewelDetector.enable();
        robot.ResetDriveEncoders();
        while (opModeIsActive()) {
            if (counter == 8) {
                robot.Suckers(RobotConstants.Suckers_Stay);
                robot.SetDrivePower(0);
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
                    robot.Drive(.35, 115, telemetry,  4);//85
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.CENTER) {
                    robot.Drive(.35, 95, telemetry,  4);//65
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.LEFT) {
                    robot.Drive(.35, 75, telemetry,  4);//45
                    counter = 5;
                }

            }

            if (counter == 5) {
                robot.EncoderTurn(Robot.Direction.CounterClockWise, .5, 95,  3);//50
                counter++;
            }
            if (counter == 6) {
                robot.Drive(.65, 5, telemetry,  3);
                counter++;

            }
            if (counter == 7) {
                robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Open);
                robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Open);
                robot.NewSuckers(RobotConstants.NewSucker_Out);
                sleep(2000);
                robot.NewSuckers(RobotConstants.NewSucker_Stay);
                counter++;

            }
            if(counter == 8){
                robot.Drive(.75,10,telemetry,3);
                counter ++;
            }
            if(counter == 9){
                robot.Drive(.65, -23, telemetry, 3);
                robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
                robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Open);
                robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Open);
                counter++;
            }
            if(counter == 10){

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
