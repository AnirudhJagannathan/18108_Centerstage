package org.firstinspires.ftc.teamcode.drive.auto;

import android.hardware.Sensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.apache.commons.math3.linear.LUDecomposition;
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
import org.firstinspires.ftc.teamcode.drive.teleop.TeleOp_DriveArvind;
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
                            .lineToLinearHeading(new Pose2d(62, -10, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(1.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(31, -41), () -> {
                                fourBar.raiseFourBar();
                                fourBar.closeClaw();
                            })
                            .addSpatialMarker(new Vector2d(40, -30), () -> {
                                slides.moveSlidesToHeightABS(0, 0.7);
                                spintake.stickIntake();
                            })
                            .build();

                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(61.25, 62, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(61.25, -5), () -> {
                                fourBar.lowerFourBar();
                                launcher.trayStickOut();
                            })
                            .addSpatialMarker(new Vector2d(62, 50), () -> {
                                fourBar.openClaw();
                                spintake.spin(true);
                            })
                            .build();

                    traj6 = drive.trajectoryBuilder(traj5.end())
                            .lineToLinearHeading(new Pose2d(62, -34, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(0.9 * DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(59, 30), () -> {
                                launcher.trayStickIn();
                            })
                            .addSpatialMarker(new Vector2d(61, 10), () -> {
                                fourBar.closeClaw();
                                spintake.outtake();
                            })
                            .addSpatialMarker(new Vector2d(61, 0), () -> {
                                fourBar.raiseFourBar();
                                slides.moveSlidesToHeightABS(950, 1.0);
                            })
                            .build();

                    traj7 = drive.trajectoryBuilder(traj6.end())
                            .lineToLinearHeading(new Pose2d(38, -45.5, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(63, -34), () -> {
                                fourBar.lowerFourBar();
                                spintake.stop();
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
                            .lineToLinearHeading(new Pose2d(62, -25, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(1.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(31, -41), () -> {
                                fourBar.raiseFourBar();
                                fourBar.closeClaw();
                            })
                            .addSpatialMarker(new Vector2d(40, -33), () -> {
                                slides.moveSlidesToHeightABS(0, 0.7);
                                spintake.stickIntake();
                            })
                            .build();

                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(61.25, 62, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(62, -5), () -> {
                                fourBar.lowerFourBar();
                                launcher.trayStickOut();
                            })
                            .addSpatialMarker(new Vector2d(62, 50), () -> {
                                fourBar.openClaw();
                                spintake.spin(true);
                            })
                            .build();

                    traj6 = drive.trajectoryBuilder(traj5.end())
                            .lineToLinearHeading(new Pose2d(63, -34, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(0.8 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(59, 25), () -> {
                                launcher.trayStickIn();
                                spintake.outtake();
                            })
                            .addSpatialMarker(new Vector2d(61, 12), () -> {
                                fourBar.closeClaw();
                            })
                            .addSpatialMarker(new Vector2d(61, 2), () -> {
                                fourBar.raiseFourBar();
                                slides.moveSlidesToHeightABS(1000, 1.0);
                            })
                            .build();

                    traj7 = drive.trajectoryBuilder(traj6.end())
                            .lineToLinearHeading(new Pose2d(40, -45.5, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(63, -34), () -> {
                                fourBar.lowerFourBar();
                                spintake.stop();
                            })
                            .build();
                } else if (pos == 3 && !posSet) {
                    /** -------------------------------------------------------------------------------------
                                                            POS = 3
                     ------------------------------------------------------------------------------------- */
                    trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                            .splineTo(new Vector2d(18, 3), Math.toRadians(-60))
                            .waitSeconds(0.1)
                            .splineToLinearHeading(new Pose2d(19, 17, Math.toRadians(0)), Math.toRadians(108))
                            .waitSeconds(0.1)
                            .splineToLinearHeading(new Pose2d(28, 17, Math.toRadians(90)), Math.toRadians(90))
                            .build();

                    traj2 = drive.trajectoryBuilder(trajSeq1.end())
                            .lineToLinearHeading(new Pose2d(30, 13, Math.toRadians(90)))
                            .build();

                    traj3 = drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(32.5, -44, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(0.8 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

                    traj4 = drive.trajectoryBuilder(traj3.end())
                            .lineToLinearHeading(new Pose2d(62, -25, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(1.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(31, -43), () -> {
                                fourBar.raiseFourBar();
                                fourBar.closeClaw();
                            })
                            .addSpatialMarker(new Vector2d(44, -33), () -> {
                                slides.moveSlidesToHeightABS(0, 0.75);
                                spintake.stickIntake();
                            })
                            .build();

                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(61.25, 62, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(62, -5), () -> {
                                fourBar.lowerFourBar();
                                launcher.trayStickOut();
                            })
                            .addSpatialMarker(new Vector2d(62, 50), () -> {
                                fourBar.openClaw();
                                spintake.spin(true);
                            })
                            .build();

                    traj6 = drive.trajectoryBuilder(traj5.end())
                            .lineToLinearHeading(new Pose2d(62, -34, Math.toRadians(91)),
                                    SampleMecanumDrive.getVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addSpatialMarker(new Vector2d(59, 25), () -> {
                                launcher.trayStickIn();
                            })
                            .addSpatialMarker(new Vector2d(61, 12), () -> {
                                fourBar.closeClaw();
                            })
                            .addSpatialMarker(new Vector2d(61, 2), () -> {
                                fourBar.raiseFourBar();
                                slides.moveSlidesToHeightABS(1000, 1.0);
                            })
                            .build();

                    traj7 = drive.trajectoryBuilder(traj6.end())
                            .lineToLinearHeading(new Pose2d(40, -45.5, Math.toRadians(91)))
                            .addSpatialMarker(new Vector2d(63, -34), () -> {
                                fourBar.lowerFourBar();
                                spintake.stop();
                            })
                            .build();
                }
            }
        }

        if (pos == 1) {
            /** -------------------------------------------------------------------------------------
                                                    POS = 1
             ------------------------------------------------------------------------------------- */

            Trajectory traj1A = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(24, -4, Math.toRadians(30)))
                    .addSpatialMarker(new Vector2d(0, 0), () -> {
                        fourBar.raiseFourBar();
                    })
                    .build();

            Trajectory traj2A = drive.trajectoryBuilder(traj1A.end())
                    .lineToLinearHeading(new Pose2d(34.5, -8, Math.toRadians(90)))
                    .build();

            Trajectory traj3A = drive.trajectoryBuilder(traj2A.end())
                    .lineToLinearHeading(new Pose2d(62.5, -12, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(50, -10), () -> {
                        slides.moveSlidesToHeightABS(350, 0.8);
                        spintake.stickOut();
                    })
                    .build();

            Trajectory traj4A = drive.trajectoryBuilder(traj3A.end())
                    .lineToLinearHeading(new Pose2d(55.65, 12.5, Math.toRadians(90)),
                            SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(62.5, -12), () -> {
                        spintake.spin(true);
                    })
                    .build();

            Trajectory traj5A = drive.trajectoryBuilder(traj4A.end())
                    .lineToLinearHeading(new Pose2d(63, -18, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(59.5, 9), () -> {
                        spintake.raiseBar();
                    })
                    .addSpatialMarker(new Vector2d(60.5, 1), () -> {
                        slides.moveSlidesToHeightABS(0, 0.8);
                    })
                    .build();

            Trajectory traj6A = drive.trajectoryBuilder(traj5A.end())
                    .lineToLinearHeading(new Pose2d(63, -68, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(63, -18), () -> {
                        fourBar.lowerFourBar();
                    })
                    .addSpatialMarker(new Vector2d(63, -28), () -> {
                        launcher.trayStickOut();
                    })
                    .addSpatialMarker(new Vector2d(63, -48), () -> {
                        fourBar.closeClaw();
                    })
                    .build();

            Trajectory traj7A = drive.trajectoryBuilder(traj6A.end())
                    .lineToLinearHeading(new Pose2d(42, -85.5, Math.toRadians(90)),
                            SampleMecanumDrive.getVelocityConstraint(0.6 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(63, -68), () -> {
                        fourBar.raiseFourBar();
                    })
                    .addSpatialMarker(new Vector2d(60, -60.5), () -> {
                        slides.moveSlidesToHeightABS(1000, 0.9);
                    })
                    .build();

            Trajectory traj8A = drive.trajectoryBuilder(traj7A.end())
                    .lineToLinearHeading(new Pose2d(42, -89.5, Math.toRadians(91)),
                            SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            waitForStart();

            drive.followTrajectory(traj1A);
            drive.followTrajectory(traj2A);

            drive.followTrajectory(traj3A);
            spintake.stickOut();
            drive.followTrajectory(traj4A);

            spintake.leftIntake();
            sleep(300);

            drive.followTrajectory(traj5A);
            launcher.trayStickIn();
            fourBar.setCollectPos(0.525, 0.325);

            drive.followTrajectory(traj6A);

            drive.followTrajectory(traj7A);

            fourBar.setCollectPos(0, 0.85);
            fourBar.lowerFourBar();
            sleep(300);

            drive.followTrajectory(traj8A);
            sleep(350);

            fourBar.openClaw();
        }

        if (pos == 2) {
            /** -------------------------------------------------------------------------------------
                                                    POS = 2
             ------------------------------------------------------------------------------------- */
            TrajectorySequence traj1B = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                    .addSpatialMarker(new Vector2d(5, 0), () -> {
                        fourBar.raiseFourBar();
                        slides.moveSlidesToHeightABS(350, 0.8);
                    })
                    .build();

            Trajectory traj2B = drive.trajectoryBuilder(traj1B.end())
                    .lineToLinearHeading(new Pose2d(38, -1, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(38, 32), () -> {
                        spintake.stickOut();
                    })
                    .build();

            Trajectory traj3B = drive.trajectoryBuilder(traj2B.end())
                    .lineToLinearHeading(new Pose2d(44.5, 12.5, Math.toRadians(90)),
                            SampleMecanumDrive.getVelocityConstraint(
                                    0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(38, 2), () -> {
                        spintake.spin(true);
                    })
                    .build();

            Trajectory traj4B = drive.trajectoryBuilder(traj3B.end())
                    .lineToLinearHeading(new Pose2d(38, -3, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(38, 9), spintake::raiseBar)
                    .addSpatialMarker(new Vector2d(38, 1), () -> {
                        fourBar.openClaw();
                        slides.moveSlidesToHeightABS(0, 0.4);
                    })
                    .build();

            Trajectory traj5B = drive.trajectoryBuilder(traj4B.end())
                    .lineToLinearHeading(new Pose2d(37, -45, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(37, -15), () -> {
                        fourBar.lowerFourBarAuto();
                        spintake.stickIn();
                    })
                    .addSpatialMarker(new Vector2d(37, -25), () -> {
                        launcher.trayStickOut();
                    })
                    .addSpatialMarker(new Vector2d(37, -43), () -> {
                        fourBar.closeClaw();
                    })
                    .build();

            Trajectory traj6B = drive.trajectoryBuilder(traj5B.end())
                    .lineToLinearHeading(new Pose2d(37, -86, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(37, -45), () -> {
                        slides.moveSlidesToHeightABS(950, 0.7);
                    })
                    .addSpatialMarker(new Vector2d(37, -80), () -> {
                        fourBar.setCollectPos(0, 0.85);
                        fourBar.lowerFourBar();
                    })
                    .build();

            Trajectory traj7B = drive.trajectoryBuilder(traj6B.end())
                    .lineToLinearHeading(new Pose2d(37, -89, Math.toRadians(90)),
                            SampleMecanumDrive.getVelocityConstraint(
                                    0.3 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            drive.followTrajectorySequence(traj1B);

            drive.followTrajectory(traj2B);
            spintake.stickOut();
            drive.followTrajectory(traj3B);

            spintake.leftIntake();
            sleep(300);

            // spintake.stickIntake();
            drive.followTrajectory(traj4B);
            launcher.trayStickIn();
            fourBar.setCollectPos(0.53, 0.32);
            drive.followTrajectory(traj5B);
            fourBar.raiseFourBar();
            drive.followTrajectory(traj6B);
            sleep(500);
            drive.followTrajectory(traj7B);

            fourBar.openClaw();
        }

        /** -------------------------------------------------------------------------------------
                                                POS = 3
         ------------------------------------------------------------------------------------- */
        if (pos == 3) {
            drive.followTrajectorySequence(trajSeq1);
        }

        fourBar.raiseFourBar();
        sleep(750);
        slides.moveSlidesToHeightABS(0, 0.7);

        webcam1.getWebcam().stopStreaming();
    }

    public void RedLeftAuto(){
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
