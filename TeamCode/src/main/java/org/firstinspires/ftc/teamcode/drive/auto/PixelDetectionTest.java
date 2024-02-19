package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.Vision.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.PixelTriangulationPipeline;
import org.firstinspires.ftc.teamcode.Vision.dataFromOpenCV;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Sensors.SensorDistance;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

@Autonomous
public class PixelDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PixelTriangulationPipeline pipeline = new PixelTriangulationPipeline();
        Webcam webcam = new Webcam(hardwareMap, "Webcam 1");
        SensorDistance distanceSensor = new SensorDistance(hardwareMap, this);
        webcam.getWebcam().openCameraDevice();

        webcam.setPipeline(pipeline);

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addData("avg1", dataFromOpenCV.AVG1W);
            telemetry.addData("avg2", dataFromOpenCV.AVG2W);
            telemetry.addData("Distance", distanceSensor.lengthDetection());
            telemetry.update();
        }

        waitForStart();
    }
}
