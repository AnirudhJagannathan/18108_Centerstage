package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

import java.util.Vector;

@Autonomous
public class BlueRight extends LinearOpMode {
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
            telemetry.addData("avg1B:", dataFromOpenCV.AVG1B);
            telemetry.addData("avg2B:", dataFromOpenCV.AVG2B);
            telemetry.addData("avg3B:", dataFromOpenCV.AVG3B);
            telemetry.update();
        }

        int pos = 0;

        if (dataFromOpenCV.AVG1B > dataFromOpenCV.AVG2B && dataFromOpenCV.AVG1B > dataFromOpenCV.AVG3B) {
            pos = 1;
        }
        if (dataFromOpenCV.AVG2B > dataFromOpenCV.AVG1B && dataFromOpenCV.AVG2B > dataFromOpenCV.AVG3B) {
            pos = 2;
        }
        if (dataFromOpenCV.AVG3B > dataFromOpenCV.AVG1B && dataFromOpenCV.AVG3B > dataFromOpenCV.AVG2B) {
            pos = 3;
        }

        //Creating Autonomous trajectory

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(27, -14, Math.toRadians(0)))
                .build();

        Trajectory traj2A = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(43, -5, Math.toRadians(90)))
                .build();

        Trajectory traj2B = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(37, 4, Math.toRadians(-95)))
                .build();

        Pose2d traj2End = traj1.end();
        if (pos == 1)
            traj2End = traj2A.end();
        if (pos == 3)
            traj2End = traj2B.end();

        Trajectory traj1Yellow = drive.trajectoryBuilder(traj2End)
                .back(48)
                .build();

        Trajectory traj2Yellow = drive.trajectoryBuilder(traj2End)
                .back(3)
                .splineTo(new Vector2d(24, -14), Math.toRadians(90))
                .back(37)
                .build();

        Trajectory traj3Yellow = drive.trajectoryBuilder(traj2End)
                .splineTo(new Vector2d(5, 0), Math.toRadians(90))
                .splineTo(new Vector2d(37, 48), Math.toRadians(90))
                .build();

        Pose2d IntakePos = traj2Yellow.end();
        if (pos == 1)
            IntakePos = traj1Yellow.end();
        if (pos == 3)
            IntakePos = traj3Yellow.end();

        Trajectory trajIntake = drive.trajectoryBuilder(IntakePos)
                .splineTo(new Vector2d(3, 1), Math.toRadians(90))
                .forward(48)
                .splineTo(new Vector2d(37, -68), Math.toRadians(90))
                .build();

        Trajectory trajDeposit = drive.trajectoryBuilder(trajIntake.end())
                .splineTo(new Vector2d(3, -48), Math.toRadians(90))
                .back(48)
                .splineTo(new Vector2d(37, 48), Math.toRadians(90))
                .build();

        Trajectory trajBack = drive.trajectoryBuilder(traj2End)
                .back(3)
                .build();

        Trajectory traj1Detected = drive.trajectoryBuilder(trajBack.end())
                .strafeLeft(27)
                .build();

        Trajectory trajPark1 = drive.trajectoryBuilder(traj1Detected.end())
                .lineToLinearHeading(new Pose2d(0, 45, Math.toRadians(95)))
                .build();

        Trajectory traj2Park = drive.trajectoryBuilder(trajBack.end())
                .lineToLinearHeading(new Pose2d(0, 45, Math.toRadians(95)))
                .build();

        Trajectory traj3Park = drive.trajectoryBuilder(trajBack.end())
                .back(35)
                .build();

        waitForStart();

        drive.followTrajectory(traj1);

       /* pos = 2;

        if (pos == 1){
            drive.followTrajectory(traj2A);
            spintake.outtake(200);
            drive.followTrajectory(traj1Detected);
            drive.followTrajectory(trajPark1);
        }
        if (pos == 2){
            drive.followTrajectory(traj2Park);
            spintake.outtake(200);
        }
        if (pos == 3){
            drive.followTrajectory(traj2B);
            spintake.outtake(200);
            drive.followTrajectory(traj3Park);
        } */


    }
}
