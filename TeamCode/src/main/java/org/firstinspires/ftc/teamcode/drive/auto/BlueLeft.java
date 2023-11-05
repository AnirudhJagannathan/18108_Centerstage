package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.Spintake;

@Autonomous
public class BlueLeft extends LinearOpMode {

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Webcam webcam1 = new Webcam(hardwareMap, "Webcam 1");
        Webcam webcam2 = new Webcam(hardwareMap, "Webcam 2");

        webcam1.setPipeline(new EasyOpenCVVision());
        webcam2.setPipeline(new EasyOpenCVVision());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam1.getWebcam(), 30);
        FtcDashboard.getInstance().startCameraStream(webcam2.getWebcam(), 30);

        waitForStart();

        while (opModeIsActive()) {

        }

        webcam1.getWebcam().stopStreaming();
    }
}
