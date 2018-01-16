package org.firstinspires.ftc.teamcode.MyCode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.OtherFiles.ClosableVuforiaLocalizer;

/**
 * Created by user on 1/16/18.
 */

public class VuMark  {

    public VuMark(){}

    private VuforiaTrackables relicTrackables = null;
    private VuforiaTrackable relicTemplate = null;
    ClosableVuforiaLocalizer vuforia;

    private RelicRecoveryVuMark OutputVuMark = RelicRecoveryVuMark.UNKNOWN;




    public void setup(){

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AS27cgT/////AAAAGacqx+RYz0dSqIWMSx+FB79tKwqL9bLs7dBcaOXHn9ZQu/mYUYWrJ2MDCHK9cKYXXKxjngTt8l0UvX854CETHuQxzI9LqSzYMyLp+Dz+hv7gV1OSZsA2mpimvfj/4mKuw7IvK8W8vBJ1IeeLkI9Zv+njNHofzdqJeMcYS35Yt/fsNSGaNo9KanFBiy4GnV1SAHSQ7qODzXN6PzbHPXw5mVWewb1XuJez0VdebTS7X5bTYzvXnMt8od4YYqIChFoLox70Jf3OKoS9IZZLUVDwBBJUG3TspO2jusJHahFWAPw5hWZkE60VUXDcfF1Pc2Q5n57FlDe5I94Sg9ca61yx6aUs42vrDPcBvq+T6mxiG52c";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforia = new ClosableVuforiaLocalizer(parameters);
        this.relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        this.relicTemplate = relicTrackables.get(0);
        this.activateVuforia();

    }


    public void activateVuforia(){
        this.relicTrackables.activate();
    }
    public void deactivateVuForia(){
        this.relicTrackables.deactivate();
    }

    public void Update(){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if(vuMark != RelicRecoveryVuMark.UNKNOWN){
            this.OutputVuMark = vuMark;
        }

    }
    public RelicRecoveryVuMark OutputVuMark(){
        return this.OutputVuMark;
    }
    public void closeVufora(){
        this.vuforia.close();
    }



}
