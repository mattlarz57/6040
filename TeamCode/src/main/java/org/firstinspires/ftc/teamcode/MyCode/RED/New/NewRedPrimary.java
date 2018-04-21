package org.firstinspires.ftc.teamcode.MyCode.RED.New;

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
 * Created by Matthew Larsen for Team 6040 The Canton RoboDogs on 4/09/18.
 */
@Autonomous(group = "NEWR")
public class NewRedPrimary extends LinearOpMode {

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


        telemetry.update();
        telemetry.addLine("Initialization: Success");


        waitForStart();
        robot.ResetDriveEncoders();

        jewelDetector.enable();
        while (opModeIsActive()) {
            if (counter == 8) {
                robot.Suckers(RobotConstants.Suckers_Stay);
                robot.SetDrivePower(0);
                // requestOpModeStop();
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

                    if (first) {
                        counter = 4;
                    }
                    if (!first) {
                        counter = 99;
                    }
                } else if (getRuntime() - Vutime >= 3.5) {
                    counter = 98;
                }
            }

            if (counter == 98) {
                if (first) {
                    robot.Drive(.65, -20, telemetry, 3);
                    robot.Camera.setPosition(RobotConstants.Camera_Jewel);
                    first = false;
                    NeedTime = true;
                    counter = 3;
                } else {
                    robot.Drive(.65, -50, telemetry, 5);
                    counter = 5;
                }

            }
            if (counter == 99) {
                if (VuMarkOutput == RelicRecoveryVuMark.RIGHT) {
                    robot.Drive(.65, -60, telemetry, 4);
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.CENTER) {
                    robot.Drive(.65, -80, telemetry, 4);
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.LEFT) {
                    robot.Drive(.65, -95, telemetry, 4);
                    counter = 5;
                }

            }

            if (counter == 4) {
                if (VuMarkOutput == RelicRecoveryVuMark.RIGHT) {
                    robot.Drive(.65, -80, telemetry, 4);
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.CENTER) {
                    robot.Drive(.65, -100, telemetry, 4);
                    counter = 5;
                } else if (VuMarkOutput == RelicRecoveryVuMark.LEFT) {
                    robot.Drive(.65, -115, telemetry, 4);
                    counter = 5;
                }

            }

            if (counter == 5) {
                robot.EncoderTurn(Robot.Direction.CounterClockWise, .85, 100, 3);
                counter++;
            }
            if (counter == 6) {
                robot.Drive(.65, 8, telemetry, 3);
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
            if (counter == 8) {
                robot.Drive(.65, 10, telemetry, 3);
                counter++;
            }
            if (counter == 9) {
                robot.Drive(.65, -23, telemetry, 3);
                robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
                robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Open);
                robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Open);
                counter++;

            }
            if (counter == 10) {
                /*robot.EncoderTurn(Robot.Direction.ClockWise, .65, 200, 3);
                robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Close_Ish);
                robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Closed_Ish);
                robot.Jeweler2.setPosition(RobotConstants.Jeweler2_Middle);
                counter++;
                                                                                      *///------------------This is where the 85 pt auto ends---------------
            }

           /* if (counter == 11) {
                if (VuMarkOutput == RelicRecoveryVuMark.RIGHT) {

                    robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Close_Ish);
                    robot.NewSuckers(RobotConstants.NewSucker_In);
                    robot.Drive(.65, 45, telemetry, 3);
                    counter = 55;

                    if (counter == 55) {
                        robot.NewSuckers(RobotConstants.NewSucker_Stay);
                        robot.Drive(.65, -40, telemetry, 3);
                        counter++;
                    }
                    if (counter == 56) {
                        robot.EncoderTurn(Robot.Direction.ClockWise, -.5, 40, 3);
                        robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Open);
                        robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Open);
                        robot.NewSuckers(RobotConstants.NewSucker_Out);
                        counter++;
                    }
                    if (counter == 57) {
                        robot.Drive(.65, 15, telemetry, 3);
                        robot.NewSuckers(RobotConstants.NewSucker_Stay);
                        counter++;
                    }
                    if (counter == 58) {
                        robot.Drive(.65, -15, telemetry, 3);
                        robot.NewSuckers(RobotConstants.NewSucker_Stay);
                        counter++;
                    }


                } else if (VuMarkOutput == RelicRecoveryVuMark.CENTER) {

                } else if (VuMarkOutput == RelicRecoveryVuMark.LEFT) {

                    robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Close_Ish);
                    robot.NewSuckers(RobotConstants.NewSucker_In);
                    robot.Drive(.65, 45, telemetry, 3);
                    counter = 65;

                    if (counter == 65) {
                        robot.NewSuckers(RobotConstants.NewSucker_Stay);
                        robot.Drive(.65, -40, telemetry, 3);
                        counter++;
                    }
                    if (counter == 66) {
                        robot.EncoderTurn(Robot.Direction.ClockWise, -.5, 230, 3);
                        robot.NewSqueezerL.setPosition(RobotConstants.NewSqueezerL_Open);
                        robot.NewSqueezerR.setPosition(RobotConstants.NewSqueezerR_Open);
                        robot.NewSuckers(RobotConstants.NewSucker_Out);
                        counter++;
                    }
                    if (counter == 67) {
                        robot.Drive(.65, 15, telemetry, 3);
                        robot.NewSuckers(RobotConstants.NewSucker_Stay);
                        counter++;
                    }
                    if (counter == 68) {
                        robot.Drive(.65, -15, telemetry, 3);
                        robot.NewSuckers(RobotConstants.NewSucker_Stay);
                        counter++;
                    }
                    }
                }
*/
            }
        }
    }



