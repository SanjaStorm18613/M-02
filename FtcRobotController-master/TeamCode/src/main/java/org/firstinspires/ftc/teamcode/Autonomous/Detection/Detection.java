package org.firstinspires.ftc.teamcode.Autonomous.Detection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.concurrent.TimeUnit;

public class Detection {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final OpenCvWebcam webcam;
    private final SquareLocationDetectorOpenCV detector;
    private final VuforiaCurrentGame vuforiaFreightFrenzy;

    private final String[] trackPos;
    private final FieldPosition[] botPos;
    private final DcMotor camLed;


    public Detection(HardwareMap hw, Telemetry tele, LinearOpMode opMode) {

        hardwareMap          = hw;
        telemetry            = tele;

        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        trackPos             = new String[]{"Blue Storage", "Blue Alliance Wall", "Red Storage", "Red Alliance Wall"};
        botPos               = new FieldPosition[]{FieldPosition.B_STORAGE, FieldPosition.B_WALL, FieldPosition.R_STORAGE, FieldPosition.R_WALL, FieldPosition.NOT_FOUND};


        int cameraMonitorViewId = opMode
                .hardwareMap
                .appContext
                .getResources()
                .getIdentifier
                        ("cameraMonitorViewId"
                                , "id"
                                , hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam
                        (hardwareMap.get
                                        (WebcamName.class, "Webcam 1")
                                , cameraMonitorViewId);

        detector = new SquareLocationDetectorOpenCV(tele);

        camLed = hw.get(DcMotor.class, "Cam");
        camLed.setDirection(DcMotorSimple.Direction.FORWARD);


        initDetectionElement();

    }


    private void initDetectionElement() {

        camLed.setPower(0.9);

        webcam.setPipeline(detector);
        webcam.setMillisecondsPermissionTimeout(2500);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                ExposureControl ec = webcam.getExposureControl();
                ec.setMode(ExposureControl.Mode.Manual);
                ec.setExposure(1, TimeUnit.MILLISECONDS);

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error while opening camera: ", errorCode);
            }
        });

    }


    public void stopDetectionElement(){

        camLed.setPower(0);

        ExposureControl ec = webcam.getExposureControl();
        ec.setExposure(5, TimeUnit.MILLISECONDS);
        ec.setMode(ExposureControl.Mode.Auto);

        webcam.stopStreaming();
    }


    public CustomElementLocation getElementDetection() {
        return detector.getLocation();
    }


    public void initVuforia () {

        vuforiaFreightFrenzy.initialize(
                "",
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                "",
                false,
                true,
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                0, 0, 0,
                AxesOrder.XZY,
                90, 90, 0,
                false
        );

        vuforiaFreightFrenzy.activate();

    }

    public void stopVuforia(){
        vuforiaFreightFrenzy.close();
    }


    public FieldPosition getBotPosition() {

        for (int i = 0; i < 4; i++) {

            if (vuforiaFreightFrenzy.track(trackPos[i]).isVisible) {

                return botPos[i];

            }
        }

        return FieldPosition.NOT_FOUND;
    }

}

