package org.firstinspires.ftc.teamcode.drive.auto;

import android.hardware.Sensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

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
import org.tensorflow.lite.support.label.LabelUtil;

@Autonomous
public class BlueRight extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    private TeleOp_DriveArvind teleOp_driveArvind;
    private SampleMecanumDrive drive;
    private Spintake spintake;
    private SensorDistance sensorDistance;
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
        teleOp_driveArvind = new TeleOp_DriveArvind();
        drive = new SampleMecanumDrive(hardwareMap);
        spintake = new Spintake(hardwareMap, this);
        sensorDistance = new SensorDistance(hardwareMap, this);
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
            {
                spintake.raiseBar();
                telemetry.addData("avg1B:", dataFromOpenCV.AVG1B);
                telemetry.addData("avg2B:", dataFromOpenCV.AVG2B);
                telemetry.addData("avg3B:", dataFromOpenCV.AVG3B);
                telemetry.update();

                /* ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if(currentDetections.size() != 0)
                {
                    boolean tagFound = false;

                    for(AprilTagDetection tag : currentDetections)
                    {
                        if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                        {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if (tagFound)
                    {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    }
                    else
                    {
                        telemetry.addLine("Don't see tag of interest :(");

                        if(tagOfInterest == null)
                        {
                            telemetry.addLine("(The tag has never been seen)");
                        }
                        else
                        {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }

                }

                telemetry.update();
                sleep(20);
                 */
            }
        }

        /* if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
         */

        int pos = 3;

        if (dataFromOpenCV.AVG1B > dataFromOpenCV.AVG2B && dataFromOpenCV.AVG1B > dataFromOpenCV.AVG3B)
            pos = 1;
        if (dataFromOpenCV.AVG2B > dataFromOpenCV.AVG1B && dataFromOpenCV.AVG2B > dataFromOpenCV.AVG3B)
            pos = 2;
        if (dataFromOpenCV.AVG3B > dataFromOpenCV.AVG1B && dataFromOpenCV.AVG3B > dataFromOpenCV.AVG2B)
            pos = 3;
        if (dataFromOpenCV.AVG1B == dataFromOpenCV.AVG2B && dataFromOpenCV.AVG2B == dataFromOpenCV.AVG3B)
            pos = 3;

        // webcam1.getWebcam().setPipeline(aprilTagDetectionPipeline);
        //Creating Autonomous trajectory

        Pose2d startPose = new Pose2d(0, 0, 0);
        Trajectory trajEnd = null;

        drive.setPoseEstimate(startPose);
        fourBar.openClaw();

        if (pos == 1) {
            /** -------------------------------------------------------------------------------------
                                                POS = 1
             ------------------------------------------------------------------------------------- */
            Trajectory traj1A = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(24, -5, Math.toRadians(25)))
                    .addSpatialMarker(new Vector2d(0, 0), () -> {
                        fourBar.raiseFourBar();
                    })
                    .build();

            Trajectory traj2A = drive.trajectoryBuilder(traj1A.end())
                    .lineToLinearHeading(new Pose2d(34.5, -9, Math.toRadians(90)))
                    .build();

            Trajectory traj3A = drive.trajectoryBuilder(traj2A.end())
                    .lineToLinearHeading(new Pose2d(35, -15, Math.toRadians(0)))
                    .addSpatialMarker(new Vector2d(35, -10), () -> {
                        slides.moveSlidesToHeightABS(350, 0.8);
                    })
                    .addSpatialMarker(new Vector2d(35, -14.5), () -> {
                        spintake.stickOut();
                    })
                    .build();

            Trajectory traj4A = drive.trajectoryBuilder(traj3A.end())
                    .lineToLinearHeading(new Pose2d(43.5, -10, Math.toRadians(-90)))
                    .build();

            Trajectory traj5A = drive.trajectoryBuilder(traj4A.end())
                    .lineToLinearHeading(new Pose2d(43.5, -12.5, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(0.8 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(43.5, -10), () -> {
                        spintake.spin();
                    })
                    .build();

            Trajectory traj6A = drive.trajectoryBuilder(traj5A.end())
                    .lineToLinearHeading(new Pose2d(63, 18, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(59.5, -7), () -> {
                        spintake.raiseBar();
                    })
                    .addSpatialMarker(new Vector2d(60.5, 7), () -> {
                        slides.moveSlidesToHeightABS(0, 0.8);
                    })
                    .build();

            Trajectory traj7A = drive.trajectoryBuilder(traj6A.end())
                    .lineToLinearHeading(new Pose2d(63, 80, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(63, 18), () -> {
                        fourBar.lowerFourBar();
                    })
                    .addSpatialMarker(new Vector2d(63, 28), () -> {
                        launcher.trayStickOut();
                    })
                    .addSpatialMarker(new Vector2d(63, 48), () -> {
                        fourBar.closeClaw();
                    })
                    .build();

            Trajectory traj8A = drive.trajectoryBuilder(traj7A.end())
                    .lineToLinearHeading(new Pose2d(30, 85.5, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(0.6 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(63, 80), () -> {
                        fourBar.raiseFourBar();
                    })
                    .addSpatialMarker(new Vector2d(60, 83), () -> {
                        slides.moveSlidesToHeightABS(1000, 0.9);
                    })
                    .build();

            Trajectory traj9A = drive.trajectoryBuilder(traj8A.end())
                    .lineToLinearHeading(new Pose2d(30, 89.5, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            trajEnd = drive.trajectoryBuilder(traj9A.end())
                    .lineToLinearHeading(new Pose2d(30, 88.5, Math.toRadians(-90)))
                    .build();

            waitForStart();

            drive.followTrajectory(traj1A);
            drive.followTrajectory(traj2A);
            sleep(300);

            drive.followTrajectory(traj3A);
            drive.followTrajectory(traj4A);
            drive.followTrajectory(traj5A);

            spintake.leftIntake();
            sleep(300);

            drive.followTrajectory(traj6A);
            launcher.trayStickIn();
            fourBar.setCollectPos(0.53, 0.32);

            drive.followTrajectory(traj7A);
            drive.followTrajectory(traj8A);
            fourBar.setCollectPos(0, 0.85);
            fourBar.lowerFourBar();
            sleep(750);
            drive.followTrajectory(traj9A);
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
                    .lineToLinearHeading(new Pose2d(38, 1, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(38, -32), () -> {
                        spintake.stickOut();
                    })
                    .build();

            Trajectory traj3B = drive.trajectoryBuilder(traj2B.end())
                    .lineToLinearHeading(new Pose2d(43, -13, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(
                                    0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(38, -2), spintake::spin)
                    .build();

            Trajectory traj4B = drive.trajectoryBuilder(traj3B.end())
                    .lineToLinearHeading(new Pose2d(38, 3, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(38, -9), spintake::raiseBar)
                    .addSpatialMarker(new Vector2d(38, -1), () -> {
                        fourBar.openClaw();
                        slides.moveSlidesToHeightABS(0, 0.4);
                    })
                    .build();

            Trajectory traj5B = drive.trajectoryBuilder(traj4B.end())
                    .lineToLinearHeading(new Pose2d(37, 45, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(37, 15), () -> {
                        fourBar.lowerFourBarAuto();
                        spintake.stickIn();
                    })
                    .addSpatialMarker(new Vector2d(37, 25), () -> {
                        launcher.trayStickOut();
                    })
                    .addSpatialMarker(new Vector2d(37, 43), () -> {
                        fourBar.closeClaw();
                    })
                    .build();

            Trajectory traj6B = drive.trajectoryBuilder(traj5B.end())
                    .lineToLinearHeading(new Pose2d(37, 86, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(37, 45), () -> {
                        slides.moveSlidesToHeightABS(950, 0.7);
                    })
                    .addSpatialMarker(new Vector2d(37, 80), () -> {
                        fourBar.setCollectPos(0, 0.85);
                        fourBar.lowerFourBar();
                    })
                    .build();

            Trajectory traj7B = drive.trajectoryBuilder(traj6B.end())
                    .lineToLinearHeading(new Pose2d(37, 89, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(
                                    0.3 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            trajEnd = drive.trajectoryBuilder(traj7B.end())
                    .lineToLinearHeading(new Pose2d(37, 87.5, Math.toRadians(-90)))
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
            Trajectory traj1C = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(24, 4, Math.toRadians(-30)))
                    .addSpatialMarker(new Vector2d(0, 0), () -> {
                        fourBar.raiseFourBar();
                    })
                    .build();

            Trajectory traj2C = drive.trajectoryBuilder(traj1C.end())
                    .lineToLinearHeading(new Pose2d(34.5, 8, Math.toRadians(-90)))
                    .build();

            Trajectory traj3C = drive.trajectoryBuilder(traj2C.end())
                    .lineToLinearHeading(new Pose2d(62.5, 12, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(50, 10), () -> {
                        slides.moveSlidesToHeightABS(350, 0.8);
                        spintake.stickOut();
                    })
                    .build();

            Trajectory traj4C = drive.trajectoryBuilder(traj3C.end())
                    .lineToLinearHeading(new Pose2d(55.65, -12.5, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(62.5, 12), () -> {
                        spintake.spin();
                    })
                    .build();

            Trajectory traj5C = drive.trajectoryBuilder(traj4C.end())
                    .lineToLinearHeading(new Pose2d(63, 18, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(59.5, -9), () -> {
                        spintake.raiseBar();
                    })
                    .addSpatialMarker(new Vector2d(60.5, -1), () -> {
                        slides.moveSlidesToHeightABS(0, 0.8);
                    })
                    .build();

            Trajectory traj6C = drive.trajectoryBuilder(traj5C.end())
                    .lineToLinearHeading(new Pose2d(63, 68, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(63, 18), () -> {
                        fourBar.lowerFourBar();
                    })
                    .addSpatialMarker(new Vector2d(63, 28), () -> {
                        launcher.trayStickOut();
                    })
                    .addSpatialMarker(new Vector2d(63, 48), () -> {
                        fourBar.closeClaw();
                    })
                    .build();

            Trajectory traj7C = drive.trajectoryBuilder(traj6C.end())
                    .lineToLinearHeading(new Pose2d(42, 85.5, Math.toRadians(-90)),
                            SampleMecanumDrive.getVelocityConstraint(0.6 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addSpatialMarker(new Vector2d(63, 68), () -> {
                        fourBar.raiseFourBar();
                    })
                    .addSpatialMarker(new Vector2d(60, 60.5), () -> {
                        slides.moveSlidesToHeightABS(1000, 0.9);
                    })
                    .build();

            Trajectory traj8C = drive.trajectoryBuilder(traj7C.end())
                    .lineToLinearHeading(new Pose2d(42, 89.5, Math.toRadians(-91)),
                            SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            trajEnd = drive.trajectoryBuilder(traj8C.end())
                    .lineToLinearHeading(new Pose2d(42, 88.5, Math.toRadians(-90)))
                    .build();

            waitForStart();

            drive.followTrajectory(traj1C);
            drive.followTrajectory(traj2C);

            drive.followTrajectory(traj3C);
            spintake.stickOut();
            drive.followTrajectory(traj4C);

            spintake.leftIntake();
            sleep(300);

            drive.followTrajectory(traj5C);
            launcher.trayStickIn();
            fourBar.setCollectPos(0.525, 0.325);

            drive.followTrajectory(traj6C);

            drive.followTrajectory(traj7C);

            fourBar.setCollectPos(0, 0.85);
            fourBar.lowerFourBar();
            sleep(300);

            drive.followTrajectory(traj8C);
            sleep(350);

            fourBar.openClaw();
        }

        drive.followTrajectory(trajEnd);

        fourBar.raiseFourBar();
        sleep(750);
        slides.moveSlidesToHeightABS(0, 0.7);

        webcam1.getWebcam().stopStreaming();
    }

    public void BlueRightAuto(){
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
