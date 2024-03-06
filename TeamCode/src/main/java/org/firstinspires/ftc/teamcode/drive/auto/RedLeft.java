package org.firstinspires.ftc.teamcode.drive.auto;
//anirudh is incredibly stupid and unintelligent
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.Vision.dataFromOpenCV;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.FourBar;
import org.firstinspires.ftc.teamcode.drive.teleop.Launcher;
import org.firstinspires.ftc.teamcode.drive.teleop.Slides;
import org.firstinspires.ftc.teamcode.drive.teleop.Spintake;
import org.firstinspires.ftc.teamcode.drive.teleop.TeleOp_Drive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous
public class RedLeft extends LinearOpMode {
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
    Trajectory traj8;
    Trajectory traj9;
    Trajectory traj10;

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
                            .lineToLinearHeading(new Pose2d(24, -1.75, Math.toRadians(30)))
                            .addSpatialMarker(new Vector2d(12, -2), () -> {
                                launcher.trayStickOut();
                            })
                            .build();

                    traj2 = drive.trajectoryBuilder(traj1.end())
                            .lineToLinearHeading(new Pose2d(34.5, -7.75, Math.toRadians(90)))
                            .build();

                    traj3 = drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(61.5, -13, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(45, -8), () -> {
                                fourBar.lowerFourBar();
                                spintake.stickIntake();
                                launcher.trayStickOut();
                            })
                            .build();

                    traj4 = drive.trajectoryBuilder(traj3.end())
                            .lineToLinearHeading(new Pose2d(61.25, 13.5, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(61.25, -10), () -> {
                                launcher.trayStickOut();
                            })
                            .addSpatialMarker(new Vector2d(61.25, 0), () -> {
                                spintake.spin(true);
                                fourBar.openClaw();
                            })
                            .build();

                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(62, -72, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(59, -30), () -> {
                                launcher.trayStickIn();
                                spintake.outtake();
                            })
                            .addSpatialMarker(new Vector2d(61, -35), () -> {
                                fourBar.closeClaw();
                            })
                            .addSpatialMarker(new Vector2d(61, -42), () -> {
                                fourBar.raiseFourBar();
                            })
                            .addSpatialMarker(new Vector2d(61, -66), () -> {
                                slides.moveSlidesToHeightABS(1050, 1.0);
                            })
                            .build();

                    traj6 = drive.trajectoryBuilder(traj5.end())
                            .lineToLinearHeading(new Pose2d(41.5, -92.75, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(48, -70), () -> {
                                fourBar.lowerFourBar();
                                spintake.stop();
                            })
                            .build();

                    traj7 = drive.trajectoryBuilder(traj6.end())
                            .lineToLinearHeading(new Pose2d(64, -60, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(60, -85), () -> {
                                fourBar.closeClaw();
                                slides.moveSlidesToHeightABS(0, 1.0);
                            })
                            .build();

                    traj8 = drive.trajectoryBuilder(traj7.end())
                            .lineToLinearHeading(new Pose2d(62, 13.5, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(61.25, -50), () -> {
                                launcher.trayStickOut();
                                fourBar.lowerFourBar();
                            })
                            .addSpatialMarker(new Vector2d(61.25, 0), () -> {
                                spintake.spin(true);
                                fourBar.openClaw();
                            })
                            .build();

                    traj9 = drive.trajectoryBuilder(traj8.end())
                            .lineToLinearHeading(new Pose2d(62, -72, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(59, -30), () -> {
                                launcher.trayStickIn();
                                spintake.outtake();
                                fourBar.closeClaw();
                            })
                            .addSpatialMarker(new Vector2d(61, -42), () -> {
                                fourBar.raiseFourBar();
                            })
                            .addSpatialMarker(new Vector2d(61, -66), () -> {
                                slides.moveSlidesToHeightABS(1050, 1.0);
                            })
                            .build();

                    traj10 = drive.trajectoryBuilder(traj9.end())
                            .lineToLinearHeading(new Pose2d(37, -92.75, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(48, -70), () -> {
                                fourBar.lowerFourBar();
                                spintake.stop();
                            })
                            .build();
                } else if (pos == 2 && !posSet) {
                    /** -------------------------------------------------------------------------------------
                     `                                      POS = 2
                     ------------------------------------------------------------------------------------- */
                    traj1 = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(35, 8.75, Math.toRadians(-30)))
                            .addSpatialMarker(new Vector2d(12, -2), () -> {
                                launcher.trayStickOut();
                            })
                            .build();

                    traj2 = drive.trajectoryBuilder(traj1.end())
                            .lineToLinearHeading(new Pose2d(38, 15.75, Math.toRadians(-90)))
                            .build();

                    traj3 = drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(61.5, 5, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(45, -8), () -> {
                                fourBar.lowerFourBar();
                                spintake.stickIntake();
                                launcher.trayStickOut();
                            })
                            .build();

                    traj4 = drive.trajectoryBuilder(traj3.end())
                            .lineToLinearHeading(new Pose2d(62, 13.5, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(61.25, -10), () -> {
                                launcher.trayStickOut();
                            })
                            .addSpatialMarker(new Vector2d(61.25, 0), () -> {
                                spintake.spin(true);
                                fourBar.openClaw();
                            })
                            .build();

                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(63.5, -72, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(61, -30), () -> {
                                launcher.trayStickIn();
                                spintake.outtake();
                            })
                            .addSpatialMarker(new Vector2d(61, -35), () -> {
                                fourBar.closeClaw();
                            })
                            .addSpatialMarker(new Vector2d(61, -42), () -> {
                                fourBar.raiseFourBar();
                            })
                            .addSpatialMarker(new Vector2d(61, -66), () -> {
                                slides.moveSlidesToHeightABS(1050, 1.0);
                            })
                            .build();

                    traj6 = drive.trajectoryBuilder(traj5.end())
                            .lineToLinearHeading(new Pose2d(36, -92.75, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(48, -70), () -> {
                                fourBar.lowerFourBar();
                                spintake.stop();
                            })
                            .build();

                    traj7 = drive.trajectoryBuilder(traj6.end())
                            .lineToLinearHeading(new Pose2d(64, -60, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(60, -85), () -> {
                                fourBar.closeClaw();
                                slides.moveSlidesToHeightABS(0, 1.0);
                            })
                            .build();

                    traj8 = drive.trajectoryBuilder(traj7.end())
                            .lineToLinearHeading(new Pose2d(62, 13.5, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(61.25, -50), () -> {
                                launcher.trayStickOut();
                                fourBar.lowerFourBar();
                            })
                            .addSpatialMarker(new Vector2d(61.25, 0), () -> {
                                spintake.spin(true);
                                fourBar.openClaw();
                            })
                            .build();

                    traj9 = drive.trajectoryBuilder(traj8.end())
                            .lineToLinearHeading(new Pose2d(63.5, -72, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(59, -30), () -> {
                                launcher.trayStickIn();
                                spintake.outtake();
                                fourBar.closeClaw();
                            })
                            .addSpatialMarker(new Vector2d(61, -42), () -> {
                                fourBar.raiseFourBar();
                            })
                            .addSpatialMarker(new Vector2d(61, -66), () -> {
                                slides.moveSlidesToHeightABS(1050, 1.0);
                            })
                            .build();

                    traj10 = drive.trajectoryBuilder(traj9.end())
                            .lineToLinearHeading(new Pose2d(33, -92.75, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(48, -70), () -> {
                                fourBar.lowerFourBar();
                                spintake.stop();
                            })
                            .build();
                } else if (pos == 3 && !posSet) {
                    /** -------------------------------------------------------------------------------------
                                                            POS = 3
                     ------------------------------------------------------------------------------------- */
                    traj1 = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(24, 1.75, Math.toRadians(-30)))
                            .addSpatialMarker(new Vector2d(12, -2), () -> {
                                launcher.trayStickOut();
                            })
                            .build();


                    traj2 = drive.trajectoryBuilder(traj1.end())
                            .lineToLinearHeading(new Pose2d(34.5, 10.75, Math.toRadians(-90)))
                            .build();

                    traj3 = drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(62.5, 5, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(45, -8), () -> {
                                fourBar.lowerFourBar();
                                spintake.stickIntake();
                                launcher.trayStickOut();
                            })
                            .build();

                    traj4 = drive.trajectoryBuilder(traj3.end())
                            .lineToLinearHeading(new Pose2d(62, 13.5, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(61.25, -10), () -> {
                                launcher.trayStickOut();
                            })
                            .addSpatialMarker(new Vector2d(61.25, 0), () -> {
                                spintake.spin(true);
                                fourBar.openClaw();
                            })
                            .build();

                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(62, -72, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(59, -30), () -> {
                                launcher.trayStickIn();
                                spintake.outtake();
                            })
                            .addSpatialMarker(new Vector2d(61, -35), () -> {
                                fourBar.closeClaw();
                            })
                            .addSpatialMarker(new Vector2d(61, -42), () -> {
                                fourBar.raiseFourBar();
                            })
                            .addSpatialMarker(new Vector2d(61, -66), () -> {
                                slides.moveSlidesToHeightABS(1050, 1.0);
                            })
                            .build();

                    traj6 = drive.trajectoryBuilder(traj5.end())
                            .lineToLinearHeading(new Pose2d(30.75, -92.75, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(48, -70), () -> {
                                fourBar.lowerFourBar();
                                spintake.stop();
                            })
                            .build();

                    traj7 = drive.trajectoryBuilder(traj6.end())
                            .lineToLinearHeading(new Pose2d(64, -60, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(60, -85), () -> {
                                fourBar.closeClaw();
                                slides.moveSlidesToHeightABS(0, 1.0);
                            })
                            .build();

                    traj8 = drive.trajectoryBuilder(traj7.end())
                            .lineToLinearHeading(new Pose2d(62, 13.5, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(61.25, -50), () -> {
                                launcher.trayStickOut();
                                fourBar.lowerFourBar();
                            })
                            .addSpatialMarker(new Vector2d(61.25, 0), () -> {
                                spintake.spin(true);
                                fourBar.openClaw();
                            })
                            .build();

                    traj9 = drive.trajectoryBuilder(traj8.end())
                            .lineToLinearHeading(new Pose2d(62, -72, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(59, -30), () -> {
                                launcher.trayStickIn();
                                spintake.outtake();
                                fourBar.closeClaw();
                            })
                            .addSpatialMarker(new Vector2d(61, -42), () -> {
                                fourBar.raiseFourBar();
                            })
                            .addSpatialMarker(new Vector2d(61, -66), () -> {
                                slides.moveSlidesToHeightABS(1150, 1.0);
                            })
                            .build();

                    traj10 = drive.trajectoryBuilder(traj9.end())
                            .lineToLinearHeading(new Pose2d(36.5, -92.95, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(48, -70), () -> {
                                fourBar.lowerFourBar();
                                spintake.stop();
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


            spintake.stickOut();
            drive.followTrajectory(traj4);

            spintake.stickIn();
            sleep(250);
            spintake.leftIntake();

            spintake.outtake();
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj6);

            fourBar.lowerFourBar();

            fourBar.openClaw();
            sleep(350);
            fourBar.raiseFourBar();

            drive.followTrajectory(traj7);

            sleep(150);
            spintake.stickOut();
            drive.followTrajectory(traj8);

            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();

            drive.followTrajectory(traj9);
            drive.followTrajectory(traj10);

            fourBar.lowerFourBar();

            fourBar.openClaw();
            sleep(350);
            fourBar.raiseFourBar();
        } else if (pos == 2) {
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);

            drive.followTrajectory(traj3);

            fourBar.openClaw();
            sleep(350);


            spintake.stickOut();
            drive.followTrajectory(traj4);


            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();

            spintake.outtake();
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj6);

            fourBar.lowerFourBar();

            fourBar.openClaw();
            sleep(350);
            fourBar.raiseFourBar();

            drive.followTrajectory(traj7);

            sleep(150);
            spintake.stickOut();
            drive.followTrajectory(traj8);

            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();

            drive.followTrajectory(traj9);
            drive.followTrajectory(traj10);

            fourBar.lowerFourBar();

            fourBar.openClaw();
            sleep(350);
            fourBar.raiseFourBar();
        } else {
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);

            drive.followTrajectory(traj3);

            fourBar.openClaw();
            sleep(350);

            spintake.stickOut();
            drive.followTrajectory(traj4);

            sleep(250);
            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();

            // spintake.outtake();
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj6);

            fourBar.lowerFourBar();

            fourBar.openClaw();
            sleep(350);
            fourBar.raiseFourBar();

            drive.followTrajectory(traj7);

            sleep(150);
            spintake.stickOut();
            drive.followTrajectory(traj8);

            sleep(250);
            spintake.stickIn();
            sleep(250);
            spintake.stickIntake();

            drive.followTrajectory(traj9);
            drive.followTrajectory(traj10);

            fourBar.lowerFourBar();

            fourBar.openClaw();
            sleep(350);
            fourBar.raiseFourBar();

        }
        slides.moveSlidesToHeightABS(1100, 0.8);
        fourBar.raiseFourBar();
        fourBar.closeClaw();
        sleep(750);
        slides.moveSlidesToHeightABS(0, 0.7);

        webcam1.getWebcam().stopStreaming();
    }

    public void RedLeftAuto() throws InterruptedException {
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
