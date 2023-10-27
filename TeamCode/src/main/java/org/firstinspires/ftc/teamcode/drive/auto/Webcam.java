package org.firstinspires.ftc.teamcode.drive.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Webcam {

    private OpenCvCamera webcam;
    private final int HEIGHT = 1920;
    private final int WIDTH = 1080;

    public Webcam(HardwareMap hardwareMap, String name) {
        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, name), cameraMonitorViewId);

        webcam.openCameraDevice();
        webcam.startStreaming(HEIGHT, WIDTH, OpenCvCameraRotation.UPRIGHT);
    }

    public void setPipeline(EasyOpenCVVision pipeline) {
        webcam.setPipeline(pipeline);
    }

    public OpenCvCamera getWebcam() {
        return webcam;
    }
}
