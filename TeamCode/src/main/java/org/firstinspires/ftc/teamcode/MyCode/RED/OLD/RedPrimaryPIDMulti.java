package org.firstinspires.ftc.teamcode.MyCode.RED.OLD;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.RobotApplication;
import org.firstinspires.ftc.teamcode.MyCode.Robot;
import org.firstinspires.ftc.teamcode.MyCode.RobotConstants;
import org.firstinspires.ftc.teamcode.OtherFiles.ClosableVuforiaLocalizer;

/**
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 3/29/18.
 */
//@Autonomous(group = "OLDR")
public class RedPrimaryPIDMulti extends LinearOpMode {

    Robot robot = new Robot();
    JewelDetector jewelDetector = new JewelDetector();
    Robot.team TeamColor = Robot.team.Red;
    ClosableVuforiaLocalizer vuforia;
    RelicRecoveryVuMark VuMarkOutput = RelicRecoveryVuMark.UNKNOWN;
    JewelDetector.JewelOrder JewelOutput = JewelDetector.JewelOrder.UNKNOWN;


    int counter;
    boolean closed = false;
    boolean disabled = false;
    boolean first = true;
    boolean NeedTime = true;
    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;
    double distance;
    ElapsedTime vutime = new ElapsedTime(0);


    @Override
    public void runOpMode() throws InterruptedException{

        telemetry.addLine("Initialization: Loading...");
        robot.initialize(hardwareMap,telemetry);
        robot.Camera.setPosition(RobotConstants.Camera_Jewel);
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA;
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;



        robot.ResetDriveEncoders();

        telemetry.update();
        telemetry.addLine("Initialization: Success");
        telemetry.update();


        waitForStart();

        jewelDetector.enable();
        while(opModeIsActive()){
            telemetry.update();
            telemetry.addData("Step: ", counter);
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if(VuMarkOutput != RelicRecoveryVuMark.UNKNOWN){
                telemetry.addData("VuMark: ", VuMarkOutput);
            }
            if (JewelOutput != JewelDetector.JewelOrder.UNKNOWN){
                telemetry.addData("Jewels: ",JewelOutput);
            }
            telemetry.update();
            if(!disabled){
                telemetry.addLine("Searching For Jewel");
                if (jewelDetector.getLastOrder() != JewelDetector.JewelOrder.UNKNOWN){
                    JewelOutput = jewelDetector.getLastOrder();
                    jewelDetector.disable();
                    robot.Camera.setPosition(RobotConstants.Camera_VuMark);
                    disabled = true;
                    counter =1;
                }
            }
            if(counter ==1){
                robot.WackJewel(TeamColor, JewelOutput);
                counter ++;
            }
            if(NeedTime && counter == 2){
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                parameters.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                vuforia = new ClosableVuforiaLocalizer(parameters);
                relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                relicTemplate = relicTrackables.get(0);
                relicTrackables.activate();
                vutime.reset();
                NeedTime = false;
            }
            else if(counter == 2){
                telemetry.addLine("Searching for VuMark");
                if(vuMark != RelicRecoveryVuMark.UNKNOWN){
                    VuMarkOutput = vuMark;
                    vuforia.close();
                    closed = true;
                    robot.Camera.setPosition(RobotConstants.Camera_Forward);

                    if(first){
                        counter = 3;
                    }
                    else{
                        counter = 99;}

                }
                else if(vutime.seconds() > 3.5){
                    counter = 98;
                }
            }
            if(counter == 98){
                if(first){
                    robot.DrivePID(.75f,-20,telemetry,3,this);
                    robot.Camera.setPosition(RobotConstants.Camera_Jewel);
                    first = false;
                    NeedTime = true;
                    counter =3;
                }
                else{
                    VuMarkOutput = RelicRecoveryVuMark.RIGHT;
                    counter = 99;
                }
            }
            if(counter == 3){
                if(VuMarkOutput == RelicRecoveryVuMark.RIGHT){
                    robot.DrivePID(.75f,-80,telemetry,4,this);
                    counter = 4;
                }
                if(VuMarkOutput == RelicRecoveryVuMark.CENTER){
                    robot.DrivePID(.75f,-100,telemetry,4,this);
                    counter =4;
                }
                if(VuMarkOutput == RelicRecoveryVuMark.LEFT){
                    robot.DrivePID(.75f,-117,telemetry,4,this);
                    counter =4;
                }
            }

            if(counter == 99){
                if(VuMarkOutput == RelicRecoveryVuMark.RIGHT){
                    robot.DrivePID(.75f,-60,telemetry,4,this);
                    counter = 4;
                }
                if(VuMarkOutput == RelicRecoveryVuMark.CENTER){
                    robot.DrivePID(.75f,-80,telemetry,4,this);
                    counter =4;
                }
                if(VuMarkOutput == RelicRecoveryVuMark.LEFT){
                    robot.DrivePID(.75f,-100,telemetry,4,this);
                    counter =4;
                }
            }
            if(counter == 4){
                robot.PIDTurn(Robot.Direction.CounterClockWise,90,.75f,telemetry,4,this);
                counter ++;

            }
            if(counter == 5){
                robot.DrivePID(.75f,20,telemetry,2,this);
                counter ++;
            }
            if(counter == 6) {
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Open);
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Open);
                robot.Suckers(RobotConstants.Suckers_Out);
                sleep(2000);
                robot.Suckers(RobotConstants.Suckers_Stay);
                robot.DrivePID(.75f, -25, telemetry, 2, this);
                counter++;
            }
            if (counter == 7){
                robot.PIDTurn(Robot.Direction.CounterClockWise,90,.75f,telemetry,3,this);
                counter ++;
            }
            if(counter == 8){
                robot.PIDTurn(Robot.Direction.CounterClockWise,90,.75f,telemetry,3,this);
                counter ++;
            }
            if(counter == 9){
                robot.DrivePID(.75f,100,telemetry,5,this);
                counter ++;
            }
            if(counter == 10){
                robot.NoCalibratePIDTurn(Robot.Direction.ClockWise,0,.75f,telemetry,1,this);
                counter ++;
            }
            if(counter == 11){
                robot.DrivePID(.75f,-100,telemetry,5,this);
                counter++;
            }
            if (counter == 12){
                robot.PIDTurn(Robot.Direction.CounterClockWise,90,.75f,telemetry,3,this);
                counter ++;
            }
            if(counter == 13){
                robot.PIDTurn(Robot.Direction.CounterClockWise,90,.75f,telemetry,3,this);
                counter ++;
            }
            if(counter == 14){
                robot.DrivePID(.75f,20,telemetry,3,this);
                counter ++;
            }
            if(counter == 15){
                robot.SqueezerL.setPosition(RobotConstants.SqueezerL_Open);
                robot.SqueezerR.setPosition(RobotConstants.SqueezerR_Open);
                robot.Suckers(RobotConstants.Suckers_Out);
                sleep(2000);
                robot.Suckers(RobotConstants.Suckers_Stay);
                robot.DrivePID(.75f, -25, telemetry, 2, this);
                counter++;
            }
            if(counter == 16){

            }





        }


    }



}
