package org.firstinspires.ftc.teamcode.drive.auto;
//anirudh is incredibly stupid and unintelligent

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.Vision.dataFromOpenCV;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.FourBar;
import org.firstinspires.ftc.teamcode.drive.teleop.Launcher;
import org.firstinspires.ftc.teamcode.drive.teleop.Slides;
import org.firstinspires.ftc.teamcode.drive.teleop.Spintake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous
public class RedRight2_0 extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    private SampleMecanumDrive drive;
    private Spintake spintake;
    private Slides slides;
    private FourBar fourBar;
    private Webcam webcam1;
    private Launcher launcher;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int pos = 0;
    int posPrev = 0;

    AprilTagDetection tagOfInterest = null;


    Trajectory traj1;
    Trajectory traj2;
    Trajectory traj3;
    Trajectory traj4;
    Trajectory traj5;
    Trajectory traj6;
    Trajectory traj7;

    TrajectorySequence trajSeq1;

    boolean posSet = false;

    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        spintake = new Spintake(hardwareMap, this);
        // SensorDistance sensorDistance = new SensorDistance(hardwareMap, this);
        slides = new Slides(hardwareMap, this);
        fourBar = new FourBar(hardwareMap, this);
        webcam1 = new Webcam(hardwareMap, "Webcam 1");
        launcher = new Launcher(hardwareMap, this);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //Webcam webcam2 = new Webcam(hardwareMap, "Webcam 2");

        webcam1.setPipeline(new EasyOpenCVVision());
        //webcam2.setPipeline(new EasyOpenCVVision());

        // FtcDashboard dashboard = FtcDashboard.getInstance();
        // telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // FtcDashboard.getInstance().startCameraStream(webcam1.getWebcam(), 30);
        // FtcDashboard.getInstance().startCameraStream(webcam2.getWebcam(), 30);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        while (opModeInInit()) {
            {
                telemetry.addData("avg1R:", dataFromOpenCV.AVG1R);
                telemetry.addData("avg2R:", dataFromOpenCV.AVG2R);
                telemetry.addData("avg3R:", dataFromOpenCV.AVG3R);
                telemetry.update();

                slides.resetSlides();
                fourBar.closeClaw();

                posPrev = pos;

                if (dataFromOpenCV.AVG1R > dataFromOpenCV.AVG2R && dataFromOpenCV.AVG1R > dataFromOpenCV.AVG3R)
                    pos = 1;
                if (dataFromOpenCV.AVG2R > dataFromOpenCV.AVG1R && dataFromOpenCV.AVG2R > dataFromOpenCV.AVG3R)
                    pos = 2;
                if (dataFromOpenCV.AVG3R > dataFromOpenCV.AVG1R && dataFromOpenCV.AVG3R > dataFromOpenCV.AVG2R)
                    pos = 3;

                posSet = (posPrev == pos);

                if (pos == 1 && !posSet) {
                    /** -------------------------------------------------------------------------------------
                                                            POS = 1
                     ------------------------------------------------------------------------------------- */
                    traj1 = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(24, -4, Math.toRadians(30)))
                            .addSpatialMarker(new Vector2d(2, -1), () -> {
                                fourBar.raiseFourBar();
                            })
                            .addSpatialMarker(new Vector2d(10, -2), () -> {
                                slides.moveSlidesToHeightABS(200, 0.7);
                            })
                            .build();

                    traj2 = drive.trajectoryBuilder(traj1.end())
                            .lineToLinearHeading(new Pose2d(34.5, -8, Math.toRadians(90)))
                            .addSpatialMarker(new Vector2d(26, -4), () -> {
                                slides.moveSlidesToHeightABS(900, 1.0);
                            })
                            .build();

                    traj3 = drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(43.5, -44, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(36, -15), () -> {
                                fourBar.lowerFourBar();
                            })
                            .build();

                    traj4 = drive.trajectoryBuilder(traj3.end())
                            .lineToLinearHeading(new Pose2d(43.5, -40, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(1.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(43.5, -43), () -> {
                                fourBar.raiseFourBar();
                                fourBar.closeClaw();
                            })

                            .build();

                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(20, -40, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(40, -40), () -> {
                                slides.moveSlidesToHeightABS(0, 0.7);
                                spintake.stickIntake();
                            })
                            .build();
                } else if (pos == 2 && !posSet) {
                    /** -------------------------------------------------------------------------------------
                                                            POS = 2
                     ------------------------------------------------------------------------------------- */
                    trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(0)))
                            .addSpatialMarker(new Vector2d(2, 0), () -> {
                                fourBar.raiseFourBar();
                                slides.moveSlidesToHeightABS(200, 0.7);
                            })
                            .lineToLinearHeading(new Pose2d(18, -3, Math.toRadians(0)))
                            .build();

                    traj2 = drive.trajectoryBuilder(trajSeq1.end())
                            .lineToLinearHeading(new Pose2d(32, -7, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(21, -4), () -> {
                                slides.moveSlidesToHeightABS(900, 1.0);
                            })
                            .build();

                    traj3 = drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(38, -45.5, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(34, -12), () -> {
                                fourBar.lowerFourBar();
                            })
                            .build();

                    traj4 = drive.trajectoryBuilder(traj3.end())
                            .lineToLinearHeading(new Pose2d(38, -40, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(1.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(38, -44), () -> {
                                fourBar.raiseFourBar();
                                fourBar.closeClaw();
                            })

                            .build();

                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(20, -40, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(40, -40), () -> {
                                slides.moveSlidesToHeightABS(0, 0.7);
                                spintake.stickIntake();
                            })
                            .build();
                } else if (pos == 3 && !posSet) {
                    /** -------------------------------------------------------------------------------------
                                                            POS = 3
                     ------------------------------------------------------------------------------------- */
                    traj1 = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(20, -13, Math.toRadians(-25)))
                            .addSpatialMarker(new Vector2d(30, -20), () -> {
                                fourBar.raiseFourBar();
                            })
                            .build();

                    traj2 = drive.trajectoryBuilder(traj1.end())
                            .lineToLinearHeading(new Pose2d(31, -36, Math.toRadians(90)))
                            .addSpatialMarker(new Vector2d(20, -16), () -> {
                                slides.moveSlidesToHeightABS(900, 0.8);
                            })
                            .build();

                    traj3 = drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(32.5, -44, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(0.8 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

                    traj4 = drive.trajectoryBuilder(traj3.end())
                            .lineToLinearHeading(new Pose2d(32.5, -40, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(1.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(32.5, -43), () -> {
                                fourBar.raiseFourBar();
                                fourBar.closeClaw();
                            })

                            .build();

                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(20, -40, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(30, -40), () -> {
                                slides.moveSlidesToHeightABS(0, 0.7);
                                spintake.stickIntake();
                            })
                            .build();
                }
            }
        }

        waitForStart();

        if (pos == 1) {
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);

            drive.followTrajectory(traj3);

            fourBar.openClaw();
            sleep(350);

            drive.followTrajectory(traj4);
            spintake.stickOut();
            drive.followTrajectory(traj5);

            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();
            sleep(250);
            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();

            drive.followTrajectory(traj6);
            drive.followTrajectory(traj7);

            fourBar.lowerFourBar();

            fourBar.openClaw();
            sleep(350);

            drive.followTrajectory(traj4);
            spintake.stickOut();
            drive.followTrajectory(traj5);

            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();
            sleep(250);
            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();

            drive.followTrajectory(traj6);
            drive.followTrajectory(traj7);

            fourBar.lowerFourBar();

            fourBar.openClaw();
            sleep(350);
        } else if (pos == 2) {
            drive.followTrajectorySequence(trajSeq1);
            drive.followTrajectory(traj2);

            drive.followTrajectory(traj3);
            fourBar.openClaw();
            sleep(350);

            drive.followTrajectory(traj4);
            drive.followTrajectory(traj5);

            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();
            sleep(250);
            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();

            drive.followTrajectory(traj6);
            drive.followTrajectory(traj7);

            fourBar.openClaw();
            sleep(350);


            drive.followTrajectory(traj4);
            spintake.stickOut();
            drive.followTrajectory(traj5);

            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();
            sleep(250);
            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();

            drive.followTrajectory(traj6);
            drive.followTrajectory(traj7);

            fourBar.openClaw();
            sleep(350);
        } else {
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);

            fourBar.lowerFourBar();

            drive.followTrajectory(traj3);

            fourBar.openClaw();
            sleep(350);

            drive.followTrajectory(traj4);
            spintake.stickOut();
            drive.followTrajectory(traj5);

            spintake.stickIn();
            spintake.stickIntake();
            spintake.stickIn();

            drive.followTrajectory(traj6);
            drive.followTrajectory(traj7);

            fourBar.lowerFourBar();

            fourBar.openClaw();
            sleep(350);

            drive.followTrajectory(traj4);
            spintake.stickOut();
            drive.followTrajectory(traj5);

            spintake.stickIn();
            spintake.stickIntake();
            spintake.stickIn();
            spintake.stickIntake();

            drive.followTrajectory(traj6);
            drive.followTrajectory(traj7);

            fourBar.lowerFourBar();

            fourBar.openClaw();
            sleep(350);

        }
        slides.moveSlidesToHeightABS(1100, 0.8);
        fourBar.raiseFourBar();
        fourBar.closeClaw();
        sleep(750);
        slides.moveSlidesToHeightABS(0, 0.7);

        webcam1.getWebcam().stopStreaming();
    }

    public void RedRightAuto() throws InterruptedException {
        runOpMode();
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
