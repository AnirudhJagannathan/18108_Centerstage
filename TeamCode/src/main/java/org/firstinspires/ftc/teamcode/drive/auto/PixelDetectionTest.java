package org.firstinspires.ftc.teamcode.drive.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.Vision.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

@Autonomous
public class PixelDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PixelDetectionPipeline pipeline = new PixelDetectionPipeline();
        Webcam webcam = new Webcam(hardwareMap, "Webcam 1");

        webcam.setPipeline(pipeline);

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addData("position:", pipeline.position);
            telemetry.addData("vals:", Arrays.toString(pipeline.vals));
            telemetry.update();
        }

        waitForStart();
    }
}
