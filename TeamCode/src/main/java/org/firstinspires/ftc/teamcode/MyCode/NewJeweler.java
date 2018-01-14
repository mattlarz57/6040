package org.firstinspires.ftc.teamcode.MyCode.TEST;


import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.MyCode.*;

import static android.graphics.Color.blue;
import static android.graphics.Color.red;

/**
 * Created by matthew on 1/13/2018.
 */

public class NewJeweler extends LinearOpMode{
    Robot robot = new Robot();
    VuforiaLocalizer vuforia;
    int blueness;
    int counter;



    @Override
    public void runOpMode() throws InterruptedException{

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
        locale.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);



        waitForStart();
        counter =1;

        while(opModeIsActive()) {


            if (counter == 1) {


                Bitmap jewelImage = OCVUtils.getVuforiaImage(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
                int Height = jewelImage.getHeight();
                int Width = jewelImage.getWidth();

                Bitmap half = Bitmap.createBitmap(Width / 2, Height, Bitmap.Config.ARGB_8888);

                for (int x = 0; x <= half.getWidth(); x++) {
                    for (int y = 0; y <= half.getHeight(); y++) {

                        if (blue(half.getPixel(x, y)) > red(half.getPixel(x, y))) {
                            blueness++;
                        }
                    }
                }

                counter ++;
            }
            while(counter ==2){
                telemetry.addData("Blueness on Left", blueness);
            }
        }






    }


}
