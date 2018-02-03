package org.firstinspires.ftc.teamcode.MyCode.TEST;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerVelocityParams;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;
import org.firstinspires.ftc.teamcode.OtherFiles.ClosableVuforiaLocalizer;

/**
 * Created by user on 1/17/18.
 */
@Autonomous
public class NewererJeweler extends OpMode {

    Robot robot = new Robot();
    JewelDetector jewelDetector = new JewelDetector();
    Robot.team TeamColor = Robot.team.Red;
    ClosableVuforiaLocalizer vuforia;
    RelicRecoveryVuMark VuMarkOutput = RelicRecoveryVuMark.UNKNOWN;
    JewelDetector.JewelOrder JewelOutput = JewelDetector.JewelOrder.UNKNOWN;

    int counter;
    boolean closed = false;
    boolean disabled = false;
    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;



    @Override
    public void init(){
        telemetry.addLine("Initialization: Loading... ");
        robot.initialize(hardwareMap,telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = new ClosableVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        telemetry.update();
        telemetry.addLine("Initialization: Success");

    }

    @Override
    public void start(){

        relicTrackables.activate();

    }

    @Override
    public void loop() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if(VuMarkOutput != RelicRecoveryVuMark.UNKNOWN){
            telemetry.addData("VuMark: ", VuMarkOutput);
            telemetry.addData("Jewel Order", JewelOutput);
        }

        telemetry.addData("VuForia Closed: ", closed);
        telemetry.update();
        if(!closed){
            telemetry.addLine("Searching For VuMark");
            if(vuMark!= RelicRecoveryVuMark.UNKNOWN){
                VuMarkOutput = vuMark;
                vuforia.close();
                closed = true;
                counter = 1;

            }

        }
        if(closed&&counter == 1){
            robot.Camera.setPosition(RobotConstants.Camera_Jewel);
            jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            jewelDetector.areaWeight = 0.02;
            jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
            jewelDetector.debugContours = true;
            jewelDetector.maxDiffrence = 15;
            jewelDetector.ratioWeight = 15;
            jewelDetector.minArea = 700;
            jewelDetector.enable();
            counter =2;
        }

        if(counter == 2 && jewelDetector.getLastOrder() != JewelDetector.JewelOrder.UNKNOWN){
            JewelOutput = jewelDetector.getLastOrder();
            jewelDetector.disable();
            disabled = true;
            counter = 3;
        }

        if (counter == 3){
            try {
                robot.WackJewel(TeamColor, JewelOutput);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            counter = 4;

        }








    }

    @Override
    public void stop(){
        vuforia.close();
        jewelDetector.disable();

    }




}
