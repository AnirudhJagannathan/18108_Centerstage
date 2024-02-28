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
import org.firstinspires.ftc.teamcode.drive.teleop.Launcher;
import org.firstinspires.ftc.teamcode.drive.teleop.Slides;
import org.firstinspires.ftc.teamcode.drive.teleop.Spintake;
import org.firstinspires.ftc.teamcode.drive.teleop.TeleOp_Drive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous
public class BlueLeftTobor extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    private SampleMecanumDrive drive;
    private Spintake spintake;
    private Slides slides;
    private FourBar fourBar;
    private Launcher launcher;
    private Webcam webcam1;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag IDs from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;

    public void runOpMode() throws InterruptedException{
        drive = new SampleMecanumDrive(hardwareMap);
        spintake = new Spintake(hardwareMap, this);
        // SensorDistance sensorDistance = new SensorDistance(hardwareMap, this);
        slides = new Slides(hardwareMap, this);
        fourBar = new FourBar(hardwareMap, this);
        launcher = new Launcher(hardwareMap, this);
        webcam1 = new Webcam(hardwareMap, "Webcam 1");
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
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

            slides.resetSlides();
            fourBar.closeClaw();
        }

        webcam1.setPipeline(new EasyOpenCVVision());

        int pos = 3;
        double avg1 = dataFromOpenCV.AVG1B;
        double avg2  = dataFromOpenCV.AVG2B;
        double avg3 = dataFromOpenCV.AVG3B;

        if (avg1 > avg2 && avg1 > avg3)
            pos = 1;
        if (avg2 > avg1 && avg2 > avg3)
            pos = 2;
        if (avg3 > avg1 && avg3 > avg2)
            pos = 3;
        if (dataFromOpenCV.AVG1B == dataFromOpenCV.AVG2B && dataFromOpenCV.AVG2B == dataFromOpenCV.AVG3B)
            pos = 3;

        telemetry.addData("avg1B:", dataFromOpenCV.AVG1B);
        telemetry.addData("avg2B:", dataFromOpenCV.AVG2B);
        telemetry.addData("avg3B:", dataFromOpenCV.AVG3B);
        telemetry.update();

        // webcam1.getWebcam().setPipeline(aprilTagDetectionPipeline);
        //Creating Autonomous trajectory

        Pose2d startPose = new Pose2d(0, 0, 0);
        TrajectorySequence trajEnd = null;

        drive.setPoseEstimate(startPose);

        if (pos == 1) {
            /** -------------------------------------------------------------------------------------
             POS = 1
             ------------------------------------------------------------------------------------- */

            Trajectory traj1A = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(20, 12, Math.toRadians(25)))
                    .addSpatialMarker(new Vector2d(30, 20), () -> {
                        fourBar.raiseFourBar();
                    })
                    .build();

            Trajectory traj2A = drive.trajectoryBuilder(traj1A.end())
                    .lineToLinearHeading(new Pose2d(31, 36, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(20, 16), () -> {
                        slides.moveSlidesToHeightABS(950, 0.8);
                    })
                    .build();

            Trajectory traj3A = drive.trajectoryBuilder(traj2A.end())
                    .lineToLinearHeading(new Pose2d(31, 41.5, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            Trajectory traj4A = drive.trajectoryBuilder(traj3A.end())
                    .lineToLinearHeading(new Pose2d(63, 25, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(31, 41), () -> {
                        fourBar.raiseFourBar(); fourBar.openClaw();
                    })
                    .addSpatialMarker(new Vector2d(40, 33), () -> {
                        slides.moveSlidesToHeightABS(50, 0.7);
                    })
                    .build();

            Trajectory traj5A = drive.trajectoryBuilder(traj4A.end())
                    .lineToLinearHeading(new Pose2d(61, -43, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(63, -20), () -> {
                        fourBar.raiseFourBar(); spintake.stickOut();
                    })
                    .build();

            Trajectory traj6A = drive.trajectoryBuilder(traj5A.end())
                    .lineToLinearHeading(new Pose2d(53, -60, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(63, -43), () -> {
                        slides.moveSlidesToHeightABS(500, 0.9);
                    })
                    .addSpatialMarker(new Vector2d(60, -50), () -> {
                        spintake.spin(true);
                    })
                    .build();

            Trajectory traj7A = drive.trajectoryBuilder(traj6A.end())
                    .lineToLinearHeading(new Pose2d(62, -30, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(60.5, -55), () -> {
                        spintake.raiseBar();
                    })
                    .addSpatialMarker(new Vector2d(60.5, -50), () -> {
                        fourBar.openClaw();
                        slides.moveSlidesToHeightABS(0, 0.8);
                    })
                    .build();

            Trajectory traj8A = drive.trajectoryBuilder(traj7A.end())
                    .lineToLinearHeading(new Pose2d(62, 34, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(62, -30), () -> {
                        fourBar.lowerFourBar();
                        spintake.stickIn();
                    })
                    .addSpatialMarker(new Vector2d(62, 0), () -> {
                        fourBar.closeClaw();
                    })
                    .build();

            Trajectory traj9A = drive.trajectoryBuilder(traj8A.end())
                    .lineToLinearHeading(new Pose2d(35, 36.5, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(0.4 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(62, 20), () -> {
                        fourBar.raiseFourBar();
                    })
                    .addSpatialMarker(new Vector2d(59, 25), () -> {
                        slides.moveSlidesToHeightABS(1050, 1.0);
                    })
                    .build();

            Trajectory traj10A = drive.trajectoryBuilder(traj9A.end())
                    .lineToLinearHeading(new Pose2d(35, 41, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(0.25 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            trajEnd = drive.trajectorySequenceBuilder(traj3A.end())
                    .lineToLinearHeading(new Pose2d(31, 40.5, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(20, 40.5, Math.toRadians(-90)))
                    .build();

            waitForStart();

            drive.followTrajectory(traj1A);
            drive.followTrajectory(traj2A);

            fourBar.lowerFourBar();

            drive.followTrajectory(traj3A);
            sleep(350);

            fourBar.openClaw();
            sleep(300);

            /* drive.followTrajectory(traj4A);
            spintake.stickOut();
            drive.followTrajectory(traj5A);
            drive.followTrajectory(traj6A);

            spintake.stickIntake();
            sleep(300);

            drive.followTrajectory(traj7A);
            drive.followTrajectory(traj8A);

            drive.followTrajectory(traj9A);

            fourBar.lowerFourBar();
            sleep(300);

            drive.followTrajectory(traj10A);
            sleep(350);

            fourBar.openClaw();
             */


        } else if (pos == 2) {
            /** -------------------------------------------------------------------------------------
             POS = 2
             ------------------------------------------------------------------------------------- */
            Trajectory traj1B = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(28, 0, Math.toRadians(0)))
                    .build();

            Trajectory traj2B = drive.trajectoryBuilder(traj1B.end())
                    .lineToLinearHeading(new Pose2d(19, 3, Math.toRadians(0)))
                    .addSpatialMarker(new Vector2d(18, 2), spintake::stickIn)
                    .build();

            Trajectory traj3B = drive.trajectoryBuilder(traj2B.end())
                    .lineToLinearHeading(new Pose2d(32, 7, Math.toRadians(-91)))
                    .build();

            Trajectory traj4B = drive.trajectoryBuilder(traj3B.end())
                    .lineToLinearHeading(new Pose2d(35, 39, Math.toRadians(-91)))
                    .build();

            Trajectory traj5B = drive.trajectoryBuilder(traj4B.end())
                    .lineToLinearHeading(new Pose2d(37, 41.5, Math.toRadians(-91)),
                            SampleMecanumDrive.getVelocityConstraint(0.25 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            Trajectory traj6B = drive.trajectoryBuilder(traj5B.end())
                    .lineToLinearHeading(new Pose2d(36, 7, Math.toRadians(-91)))
                    .addSpatialMarker(new Vector2d(37, 37), () -> {
                        slides.moveSlidesToHeightABS(50, 0.8);
                    })
                    .build();

            Trajectory traj7B = drive.trajectoryBuilder(traj6B.end())
                    .lineToLinearHeading(new Pose2d(36, -47, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(40, -32), () -> {
                        spintake.stickOut();
                        fourBar.raiseFourBar();
                        slides.moveSlidesToHeightABS(500, 0.9);
                    })
                    .build();

            Trajectory traj8B = drive.trajectoryBuilder(traj7B.end())
                    .lineToLinearHeading(new Pose2d(42, -59.75, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(40, -50), () -> {
                        spintake.spin(true);
                    })
                    .build();

            Trajectory traj9B = drive.trajectoryBuilder(traj8B.end())
                    .lineToLinearHeading(new Pose2d(36, -50, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(40, -42), spintake::raiseBar)
                    .addSpatialMarker(new Vector2d(40, -35), () -> {
                        fourBar.openClaw();
                        slides.moveSlidesToHeightABS(0, 0.3);
                    })
                    .build();


            Trajectory traj10B = drive.trajectoryBuilder(traj9B.end())
                    .lineToLinearHeading(new Pose2d(35, 39, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(37, -5), fourBar::lowerFourBar)
                    .build();

            Trajectory traj11B = drive.trajectoryBuilder(traj10B.end())
                    .lineToLinearHeading(new Pose2d(35, 41, Math.toRadians(-90)))
                    .build();

            trajEnd = drive.trajectorySequenceBuilder(traj5B.end())
                    .lineToLinearHeading(new Pose2d(37, 40.5, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(20, 40.5, Math.toRadians(-90)))
                    .build();

            waitForStart();

            drive.followTrajectory(traj1B);
            drive.followTrajectory(traj2B);

            sleep(400);

            drive.followTrajectory(traj3B);
            drive.followTrajectory(traj4B);

            sleep(500);

            fourBar.closeClaw();
            sleep(500);
            fourBar.raiseFourBar();
            sleep(300);
            slides.moveSlidesToHeightABS(850, 0.7);
            sleep(300);
            fourBar.lowerFourBar();
            sleep(500);

            drive.followTrajectory(traj5B);

            fourBar.openClaw();
            sleep(300);

            /* fourBar.raiseFourBar();
            sleep(300);

            drive.followTrajectory(traj6B);
            drive.followTrajectory(traj7B);
            drive.followTrajectory(traj8B);

            spintake.stickIntake();
            sleep(500);
            // spintake.stickIntake();
            drive.followTrajectory(traj9B);
            launcher.trayStickIn();
            fourBar.setCollectPos(0.53, 0.32);
            drive.followTrajectory(traj10B);

            fourBar.setCollectPos(0, 0.85);
            fourBar.lowerFourBar();
            sleep(400);
            launcher.trayStickOut();
            sleep(250);
            fourBar.closeClaw();
            sleep(500);
            fourBar.raiseFourBar();
            sleep(500);
            slides.moveSlidesToHeightABS(1150, 0.7);
            sleep(300);
            fourBar.lowerFourBar();
            sleep(750);

            drive.followTrajectory(traj11B);

            fourBar.openClaw();
             */
        } else {
            /** -------------------------------------------------------------------------------------
             POS = 3
             ------------------------------------------------------------------------------------- */
            Trajectory traj1C = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(24, 2, Math.toRadians(-25)))
                    .build();

            Trajectory traj2C = drive.trajectoryBuilder(traj1C.end())
                    .lineToLinearHeading(new Pose2d(34.5, 8, Math.toRadians(-90)))
                    .build();

            Trajectory traj3C = drive.trajectoryBuilder(traj2C.end())
                    .lineToLinearHeading(new Pose2d(40, 38, Math.toRadians(-91)))
                    .build();

            Trajectory traj4C = drive.trajectoryBuilder(traj3C.end())
                    .lineToLinearHeading(new Pose2d(44, 40.5, Math.toRadians(-91)),
                            SampleMecanumDrive.getVelocityConstraint(0.4 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            Trajectory traj5C = drive.trajectoryBuilder(traj4C.end())
                    .lineToLinearHeading(new Pose2d(64, 0, Math.toRadians(-91)),
                            SampleMecanumDrive.getVelocityConstraint(0.8 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(47, 35), () -> {
                        slides.moveSlidesToHeightABS(50, 0.7);
                    })
                    .build();

            Trajectory traj6C = drive.trajectoryBuilder(traj5C.end())
                    .lineToLinearHeading(new Pose2d(61.5, -43, Math.toRadians(-91)))
                    .addSpatialMarker(new Vector2d(63, -20), () -> {
                        fourBar.raiseFourBar();
                        spintake.stickOut();
                    })
                    .build();

            Trajectory traj7C = drive.trajectoryBuilder(traj6C.end())
                    .lineToLinearHeading(new Pose2d(52.5, -59.75, Math.toRadians(-91)),
                            SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(62, -43), () -> {
                        slides.moveSlidesToHeightABS(500, 0.9);
                    })
                    .addSpatialMarker(new Vector2d(59, 50), () -> {
                        spintake.spin(true);
                    })
                    .build();

            Trajectory traj8C = drive.trajectoryBuilder(traj7C.end())
                    .lineToLinearHeading(new Pose2d(64, -30, Math.toRadians(-91)))
                    .addSpatialMarker(new Vector2d(57, -57), () -> {
                        spintake.raiseBar();
                    })
                    .addSpatialMarker(new Vector2d(61, -50), () -> {
                        fourBar.openClaw();
                        slides.moveSlidesToHeightABS(0, 0.8);
                    })
                    .build();

            Trajectory traj9C = drive.trajectoryBuilder(traj8C.end())
                    .lineToLinearHeading(new Pose2d(64, 20, Math.toRadians(-91)))
                    .addSpatialMarker(new Vector2d(64, -30), () -> {
                        fourBar.lowerFourBar();
                    })
                    .addSpatialMarker(new Vector2d(64, 0), () -> {
                        fourBar.closeClaw();
                    })
                    .build();

            Trajectory traj10C = drive.trajectoryBuilder(traj9C.end())
                    .lineToLinearHeading(new Pose2d(46, 37, Math.toRadians(-91)),
                            SampleMecanumDrive.getVelocityConstraint(0.6 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(64, 20), () -> {
                        fourBar.raiseFourBar();
                    })
                    .addSpatialMarker(new Vector2d(60, 22.5), () -> {
                        slides.moveSlidesToHeightABS(1050, 0.8);
                    })
                    .build();

            Trajectory traj11C = drive.trajectoryBuilder(traj10C.end())
                    .lineToLinearHeading(new Pose2d(44, 40.5, Math.toRadians(-91)),
                            SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            trajEnd = drive.trajectorySequenceBuilder(traj4C.end())
                    .lineToLinearHeading(new Pose2d(44, 39, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(20, 40.5, Math.toRadians(-90)))
                    .build();

            waitForStart();

            drive.followTrajectory(traj1C);
            drive.followTrajectory(traj2C);

            drive.followTrajectory(traj3C);

            fourBar.closeClaw();
            sleep(500);
            fourBar.raiseFourBar();
            sleep(300);
            slides.moveSlidesToHeightABS(950, 0.7);
            sleep(300);
            fourBar.lowerFourBar();
            sleep(750);

            drive.followTrajectory(traj4C);
            sleep(300);

            fourBar.openClaw();
            sleep(300);

            /* fourBar.raiseFourBar();
            sleep(750);

            drive.followTrajectory(traj5C);

            spintake.stickOut();
            drive.followTrajectory(traj6C);
            drive.followTrajectory(traj7C);

            spintake.stickIntake();
            sleep(300);

            drive.followTrajectory(traj8C);
            drive.followTrajectory(traj9C);

            drive.followTrajectory(traj10C);

            fourBar.lowerFourBar();
            sleep(300);

            drive.followTrajectory(traj11C);
            sleep(350);

            fourBar.openClaw();
             */
        }

        drive.followTrajectorySequence(trajEnd);

        fourBar.raiseFourBar();
        sleep(750);
        slides.moveSlidesToHeightABS(0, 0.7);

        webcam1.getWebcam().stopStreaming();
    }

    public void BlueLeftAuto(){
        waitForStart();

        drive.setMotorPowers(0.1, 0.1,0.1, 0.1);
        sleep(250);
        drive.setMotorPowers(0,0,0,0);

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
