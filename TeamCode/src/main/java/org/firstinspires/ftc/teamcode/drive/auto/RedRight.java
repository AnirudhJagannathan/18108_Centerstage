package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.Vision.dataFromOpenCV;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Sensors.SensorDistance;
import org.firstinspires.ftc.teamcode.drive.teleop.FourBar;
import org.firstinspires.ftc.teamcode.drive.teleop.Slides;
import org.firstinspires.ftc.teamcode.drive.teleop.Spintake;

@Autonomous
public class RedRight extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Spintake spintake = new Spintake(hardwareMap, this);
        SensorDistance sensorDistance = new SensorDistance(hardwareMap, this);
        Slides slides = new Slides(hardwareMap, this);
        FourBar fourBar = new FourBar(hardwareMap, this);
        Webcam webcam1 = new Webcam(hardwareMap, "Webcam 1");
        //Webcam webcam2 = new Webcam(hardwareMap, "Webcam 2");

        webcam1.setPipeline(new EasyOpenCVVision());
        //webcam2.setPipeline(new EasyOpenCVVision());

        // FtcDashboard dashboard = FtcDashboard.getInstance();
        // telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // FtcDashboard.getInstance().startCameraStream(webcam1.getWebcam(), 30);
        // FtcDashboard.getInstance().startCameraStream(webcam2.getWebcam(), 30);

        while (opModeInInit()) {
            telemetry.addData("avg1B:", dataFromOpenCV.AVG1R);
            telemetry.addData("avg2B:", dataFromOpenCV.AVG2R);
            telemetry.addData("avg3B:", dataFromOpenCV.AVG3R);
            telemetry.update();
        }

        int pos = 0;

        if (dataFromOpenCV.AVG1R > dataFromOpenCV.AVG2R && dataFromOpenCV.AVG1R > dataFromOpenCV.AVG3R) {
            pos = 1;
        }
        if (dataFromOpenCV.AVG2R > dataFromOpenCV.AVG1R && dataFromOpenCV.AVG2R > dataFromOpenCV.AVG3R) {
            pos = 2;
        }
        if (dataFromOpenCV.AVG3R > dataFromOpenCV.AVG1R && dataFromOpenCV.AVG3R > dataFromOpenCV.AVG2R) {
            pos = 3;
        }

        //Creating Autonomous trajectory

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(5, 14, Math.toRadians(0)))
                .build();

        Trajectory traj2A = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(43, -5, Math.toRadians(90)))
                .build();

        Trajectory traj2B = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(37, 4, Math.toRadians(-95)))
                .build();

        Trajectory trajBack = drive.trajectoryBuilder(traj1.end())
                .back(3)
                .build();

        Trajectory trajPark2 = drive.trajectoryBuilder(trajBack.end())
                .strafeRight(36)
                .build();

       /* pos = 2;

        Pose2d traj2End = traj1.end();
        if (pos == 1)
            traj2End = traj2A.end();
        if (pos == 3)
            traj2End = traj2B.end(); */

        waitForStart();

        drive.followTrajectory(traj1);
    }
}
