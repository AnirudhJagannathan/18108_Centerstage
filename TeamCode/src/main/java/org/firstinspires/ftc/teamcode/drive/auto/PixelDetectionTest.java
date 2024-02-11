package org.firstinspires.ftc.teamcode.drive.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.Vision.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.PixelTriangulationPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

@Autonomous
public class PixelDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PixelTriangulationPipeline pipeline = new PixelTriangulationPipeline();
        Webcam webcam = new Webcam(hardwareMap, "Webcam 1");

        webcam.getWebcam().openCameraDevice();

        webcam.setPipeline(pipeline);

        while (opModeInInit() && !isStopRequested()) {
            // webcam.getWebcam().startStreaming(pipeline.getRows(), pipeline.getCols(), OpenCvCameraRotation.UPRIGHT);
        }

        waitForStart();
    }
}
