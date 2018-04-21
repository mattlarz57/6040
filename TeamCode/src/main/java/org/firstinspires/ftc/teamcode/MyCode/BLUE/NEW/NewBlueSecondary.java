package org.firstinspires.ftc.teamcode.MyCode.BLUE.NEW;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.disnodeteam.dogecv.math.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;
import org.firstinspires.ftc.teamcode.OtherFiles.AutoTransitioner;
import org.firstinspires.ftc.teamcode.OtherFiles.ClosableVuforiaLocalizer;

/**
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 4/11/18.
 */
//@Autonomous(group = "NEWB")
public class NewBlueSecondary extends LinearOpMode {

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
    double Vutime;


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
        jewelDetector.rotateMat = true;
        AutoTransitioner.transitionOnStop(this, "FinalTele");
        telemetry.update();
        telemetry.addLine("Initialization: Complete");


        counter = 1;
        waitForStart();
        ElapsedTime elapsedTime = new ElapsedTime(0);
        jewelDetector.enable();
        robot.ResetDriveEncoders();
        while (opModeIsActive()){
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
            if(counter == 3 && NeedTime){
                Vutime = getRuntime();
                NeedTime = false;
            }


            else if (counter == 3) {
                telemetry.addLine("Searching for VuMark");
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    VuMarkOutput = vuMark;
                    vuforia.close();
                    closed = true;
                    robot.Camera.setPosition(RobotConstants.Camera_Forward);
                    counter = 4;


                }
                else if(getRuntime() - Vutime >= 3.5){
                    VuMarkOutput = RelicRecoveryVuMark.RIGHT;
                    counter = 4;
                }
            }


            if(counter == 4){
                robot.Drive(.35,70,telemetry, 4);
                counter ++;
            }

            if(counter == 5){
                robot.EncoderTurn(Robot.Direction.ClockWise,.35,100, 3);
                counter ++;
            }
            if(counter == 6){
                if (VuMarkOutput == RelicRecoveryVuMark.LEFT) {
                    robot.Drive(.35,7,telemetry, 2);
                    counter = 7;
                } else if (VuMarkOutput == RelicRecoveryVuMark.CENTER) {
                    robot.Drive(.35, 22,telemetry, 3);
                    counter = 7;
                } else if (VuMarkOutput == RelicRecoveryVuMark.RIGHT) {
                    robot.Drive(.35,42,telemetry, 3);
                    counter = 7;
                }

            }
            if(counter == 7){
                robot.EncoderTurn(Robot.Direction.CounterClockWise, .5, 65, 3);
                counter++;
            }
            if(counter == 8){
                robot.Drive(.35,12,telemetry, 3);
                counter++;

            }
            if(counter == 9){
                robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Open);
                robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Open);
                robot.NewSuckers(RobotConstants.NewSucker_Out);
                sleep(2000);
                robot.NewSuckers(RobotConstants.NewSucker_Stay);
                robot.Drive(.35,-14,telemetry, 3);
                counter ++;
            }
            if(counter == 10){
            }

        }

    }
}
