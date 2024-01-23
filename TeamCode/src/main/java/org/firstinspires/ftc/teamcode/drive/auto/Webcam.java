package org.firstinspires.ftc.teamcode.drive.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.Vision.PixelDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Webcam {

    private OpenCvCamera webcam;
    private final int HEIGHT = 720;
    private final int WIDTH = 1280;

    public Webcam(HardwareMap hardwareMap, String name) {
        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, name), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void setPipeline(EasyOpenCVVision pipeline) {
        webcam.setPipeline(pipeline);
    }

    public void setPipeline(PixelDetectionPipeline pipeline) {
        webcam.setPipeline(pipeline);
    }

    public OpenCvCamera getWebcam() {
        return webcam;
    }
}
