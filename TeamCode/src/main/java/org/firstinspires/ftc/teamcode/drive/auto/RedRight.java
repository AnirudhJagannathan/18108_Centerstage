package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.Vision.dataFromOpenCV;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Sensors.SensorDistance;
import org.firstinspires.ftc.teamcode.drive.teleop.FourBar;
import org.firstinspires.ftc.teamcode.drive.teleop.Slides;
import org.firstinspires.ftc.teamcode.drive.teleop.Spintake;
import org.firstinspires.ftc.teamcode.drive.teleop.TeleOp_Drive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous
public class RedRight extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag IDs from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;

    public void runOpMode() throws InterruptedException{
        TeleOp_Drive teleOp_drive = new TeleOp_Drive();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Spintake spintake = new Spintake(hardwareMap, this);
        // SensorDistance sensorDistance = new SensorDistance(hardwareMap, this);
        Slides slides = new Slides(hardwareMap, this);
        FourBar fourBar = new FourBar(hardwareMap, this);
        Webcam webcam1 = new Webcam(hardwareMap, "Webcam 1");
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //Webcam webcam2 = new Webcam(hardwareMap, "Webcam 2");

        webcam1.setPipeline(new EasyOpenCVVision());
        //webcam2.setPipeline(new EasyOpenCVVision());

        // FtcDashboard dashboard = FtcDashboard.getInstance();
        // telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // FtcDashboard.getInstance().startCameraStream(webcam1.getWebcam(), 30);
        // FtcDashboard.getInstance().startCameraStream(webcam2.getWebcam(), 30);

        while (opModeInInit()) {
            {
                telemetry.addData("avg1R:", dataFromOpenCV.AVG1R);
                telemetry.addData("avg2R:", dataFromOpenCV.AVG2R);
                telemetry.addData("avg3R:", dataFromOpenCV.AVG3R);
                telemetry.update();

                slides.resetSlides();
                fourBar.closeClaw();
            }
        }

        int pos = 3;

        if (dataFromOpenCV.AVG1R > dataFromOpenCV.AVG2R && dataFromOpenCV.AVG1R > dataFromOpenCV.AVG3R)
            pos = 1;
        if (dataFromOpenCV.AVG2R > dataFromOpenCV.AVG1R && dataFromOpenCV.AVG2R > dataFromOpenCV.AVG3R)
            pos = 2;
        if (dataFromOpenCV.AVG3R > dataFromOpenCV.AVG1R && dataFromOpenCV.AVG3R > dataFromOpenCV.AVG2R)
            pos = 3;
        if (dataFromOpenCV.AVG1R == dataFromOpenCV.AVG2R && dataFromOpenCV.AVG2R == dataFromOpenCV.AVG3R)
            pos = 3;

        // webcam1.getWebcam().setPipeline(aprilTagDetectionPipeline);
        //Creating Autonomous trajectory

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        /** -------------------------------------------------------------------------------------
                                            POS = 1
         ------------------------------------------------------------------------------------- */
        Trajectory traj1A = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(0)))
                .build();

        Trajectory traj2A = drive.trajectoryBuilder(traj1A.end())
                .lineToLinearHeading(new Pose2d(40, -8, Math.toRadians(90)))
                .build();

        Trajectory traj3A = drive.trajectoryBuilder(traj2A.end())
                .lineToLinearHeading(new Pose2d(32, -5, Math.toRadians(91)))
                .build();

        Trajectory traj4A = drive.trajectoryBuilder(traj3A.end())
                .lineToLinearHeading(new Pose2d(35, -40, Math.toRadians(91)))
                .build();

        Trajectory traj5A = drive.trajectoryBuilder(traj4A.end())
                .lineToLinearHeading(new Pose2d(40, -42, Math.toRadians(91)),
                        SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        /** -------------------------------------------------------------------------------------
                                            POS = 2
            ------------------------------------------------------------------------------------- */
        Trajectory traj1B = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .build();

        Trajectory traj2B = drive.trajectoryBuilder(traj1B.end())
                .lineToLinearHeading(new Pose2d(19, -3, Math.toRadians(0)))
                .build();

        Trajectory traj3B = drive.trajectoryBuilder(traj2B.end())
                .lineToLinearHeading(new Pose2d(32, -7, Math.toRadians(91)))
                .build();

        Trajectory traj4B = drive.trajectoryBuilder(traj3B.end())
                .lineToLinearHeading(new Pose2d(35, -39, Math.toRadians(91)))
                .build();

        Trajectory traj5B = drive.trajectoryBuilder(traj4B.end())
                .lineToLinearHeading(new Pose2d(37.5, -41.5, Math.toRadians(91)),
                        SampleMecanumDrive.getVelocityConstraint(0.25 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj6B = drive.trajectoryBuilder(traj5B.end())
                .lineToLinearHeading(new Pose2d(37, -7, Math.toRadians(91)))
                .addSpatialMarker(new Vector2d(37, -37), () -> {
                    slides.moveSlidesToHeightABS(50, 0.6);
                    spintake.stickIn();
                })
                .build();

        Trajectory traj7B = drive.trajectoryBuilder(traj6B.end())
                .lineToLinearHeading(new Pose2d(40, 50, Math.toRadians(92)))
                .addSpatialMarker(new Vector2d(40, 35), () -> {
                    spintake.stickOut();
                    fourBar.raiseFourBar();
                    slides.moveSlidesToHeightABS(550, 0.8);
                })
                .build();

        Trajectory traj8B = drive.trajectoryBuilder(traj7B.end())
                .lineToLinearHeading(new Pose2d(47.5, 60.75, Math.toRadians(92)),
                        SampleMecanumDrive.getVelocityConstraint(0.4 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addSpatialMarker(new Vector2d(40, 50), spintake::spin)
                .build();

        Trajectory traj9B = drive.trajectoryBuilder(traj8B.end())
                .lineToLinearHeading(new Pose2d(40, 50, Math.toRadians(92)))
                .addSpatialMarker(new Vector2d(40, 42), spintake::raiseBar)
                .addSpatialMarker(new Vector2d(40, 35), () -> {
                    fourBar.openClaw();
                    slides.moveSlidesToHeightABS(50, 0.3);
                })
                .build();

        Trajectory traj10B = drive.trajectoryBuilder(traj9B.end())
                .lineToLinearHeading(new Pose2d(37, -39, Math.toRadians(92)))
                .addDisplacementMarker(fourBar::lowerFourBar)
                .build();

        Trajectory traj11B = drive.trajectoryBuilder(traj10B.end())
                .lineToLinearHeading(new Pose2d(37, -43, Math.toRadians(92)))
                .build();


        /** -------------------------------------------------------------------------------------
                                            POS = 3
         ------------------------------------------------------------------------------------- */
        Trajectory traj1C = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(0)))
                .build();

        Trajectory traj2C = drive.trajectoryBuilder(traj1C.end())
                .lineToLinearHeading(new Pose2d(40, 8, Math.toRadians(-90)))
                .build();

        TrajectorySequence traj3C = drive.trajectorySequenceBuilder(traj2C.end())
                .lineToLinearHeading(new Pose2d(24, 5, Math.toRadians(91)))
                //.turn(Math.toRadians(180))
                .build();

        Trajectory traj4C = drive.trajectoryBuilder(traj3C.end())
                .lineToLinearHeading(new Pose2d(20, -35, Math.toRadians(91)))
                .build();

        Trajectory traj5C = drive.trajectoryBuilder(traj4C.end())
                .lineToLinearHeading(new Pose2d(31, -42, Math.toRadians(91)),
                        SampleMecanumDrive.getVelocityConstraint(0.8 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        waitForStart();

        spintake.stickOut();

        if (pos == 1) {
            drive.followTrajectory(traj1A);
            drive.followTrajectory(traj2A);

            spintake.outtake(0.3, 500);
            spintake.stop();
            sleep(400);

            drive.followTrajectory(traj3A);
            drive.followTrajectory(traj4A);

            sleep(750);

            fourBar.closeClaw();
            sleep(750);
            fourBar.raiseFourBar();
            sleep(500);
            slides.moveSlidesToHeightABS(1150, 0.7);
            sleep(300);
            fourBar.lowerFourBar();
            sleep(1000);

            drive.followTrajectory(traj5A);

            fourBar.openClaw();
        } else if (pos == 2) {
            drive.followTrajectory(traj1B);

            spintake.outtake(0.4, 500);
            spintake.stop();
            sleep(200);

            drive.followTrajectory(traj2B);

            sleep(400);

            drive.followTrajectory(traj3B);
            drive.followTrajectory(traj4B);

            sleep(500);

            fourBar.closeClaw();
            sleep(500);
            fourBar.raiseFourBar();
            sleep(300);
            slides.moveSlidesToHeightABS(1000, 0.7);
            sleep(300);
            fourBar.lowerFourBar();
            sleep(500);

            drive.followTrajectory(traj5B);

            fourBar.openClaw();

            fourBar.raiseFourBar();
            sleep(300);

            drive.followTrajectory(traj6B);
            drive.followTrajectory(traj7B);
            drive.followTrajectory(traj8B);

            spintake.stickIntake();
            sleep(300);
            // spintake.stickIntake();
            drive.followTrajectory(traj9B);
            drive.followTrajectory(traj10B);

            fourBar.closeClaw();
            sleep(500);
            fourBar.raiseFourBar();
            sleep(300);
            slides.moveSlidesToHeightABS(1000, 0.7);
            sleep(300);
            fourBar.lowerFourBar();
            sleep(500);

             drive.followTrajectory(traj11B);

             fourBar.openClaw();
        } else {
            drive.followTrajectory(traj1C);
            drive.followTrajectory(traj2C);

            spintake.outtake(0.4, 500);
            spintake.stop();
            sleep(400);

            drive.followTrajectorySequence(traj3C);
            drive.followTrajectory(traj4C);

            sleep(750);

            fourBar.closeClaw();
            sleep(750);
            fourBar.raiseFourBar();
            sleep(500);
            slides.moveSlidesToHeightABS(1150, 0.7);
            sleep(300);
            fourBar.lowerFourBar();
            sleep(1000);

            drive.followTrajectory(traj5C);

            fourBar.openClaw();
        }

        fourBar.raiseFourBar();
        sleep(750);
        slides.moveSlidesToHeightABS(0, 0.7);

        webcam1.getWebcam().stopStreaming();
    }

    private Object dataFromOpenCV() {
        return dataFromOpenCV();
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        /* telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.ftcPose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.ftcPose.roll)));
         */
    }
}
