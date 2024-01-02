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
import org.firstinspires.ftc.teamcode.drive.teleop.Slides;
import org.firstinspires.ftc.teamcode.drive.teleop.Spintake;
import org.firstinspires.ftc.teamcode.drive.teleop.TeleOp_Drive;
import org.firstinspires.ftc.teamcode.drive.teleop.TeleOp_DriveArvind;
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
    private SensorDistance sensorDistance;
    private Slides slides;
    private FourBar fourBar;
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
        sensorDistance = new SensorDistance(hardwareMap, this);
        slides = new Slides(hardwareMap, this);
        fourBar = new FourBar(hardwareMap, this);
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

        drive.setPoseEstimate(startPose);

        /** -------------------------------------------------------------------------------------
                                                POS = 1
         ------------------------------------------------------------------------------------- */

        Trajectory traj1A = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(0)))
                .build();

        Trajectory traj2A = drive.trajectoryBuilder(traj1A.end())
                .lineToLinearHeading(new Pose2d(40, -8, Math.toRadians(-90)))
                .build();

        Trajectory traj3A = drive.trajectoryBuilder(traj2A.end())
                .lineToLinearHeading(new Pose2d(65, 12, Math.toRadians(91)))
                .build();

        /*Trajectory traj3mid = drive.trajectoryBuilder(traj1.end())

                .build();*/

        Trajectory traj4A = drive.trajectoryBuilder(traj3A.end())
                .lineToLinearHeading(new Pose2d(65, 68, Math.toRadians(91)))
                .build();

        Trajectory traj5A = drive.trajectoryBuilder(traj4A.end())
                .lineToLinearHeading(new Pose2d(38, 80, Math.toRadians(91)),
                        SampleMecanumDrive.getVelocityConstraint(0.25 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj6A = drive.trajectoryBuilder(traj5A.end().plus(new Pose2d(0, -5, Math.toRadians(0))))
                .lineToLinearHeading(new Pose2d(8, 50, Math.toRadians(91)))
                .build();

        Trajectory traj7A = drive.trajectoryBuilder(traj6A.end())
                .lineToLinearHeading(new Pose2d(8, 0, Math.toRadians(91)))
                .build();

        Trajectory traj8A = drive.trajectoryBuilder(traj7A.end())
                .lineToLinearHeading(new Pose2d(37, -32, Math.toRadians(91)),
                        SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj9A = drive.trajectoryBuilder(traj8A.end())
                .lineToLinearHeading(new Pose2d(65, 0, Math.toRadians(91)))
                .build();

        Trajectory traj10A = drive.trajectoryBuilder(traj9A.end())
                .lineToLinearHeading(new Pose2d(65, 68, Math.toRadians(91)))
                .build();

        Trajectory traj11A = drive.trajectoryBuilder(traj10A.end())
                .lineToLinearHeading(new Pose2d(35, 80, Math.toRadians(91)))
                .build();

        /** -------------------------------------------------------------------------------------
                                                POS = 2
         ------------------------------------------------------------------------------------- */

        Trajectory traj1B = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .build();

        Trajectory backwardsB = drive.trajectoryBuilder(traj1B.end())
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(0)))
                .build();

        Trajectory traj3B = drive.trajectoryBuilder(backwardsB.end())
                .lineToLinearHeading(new Pose2d(34, -1, Math.toRadians(-91)))
                .build();

        Trajectory traj4B = drive.trajectoryBuilder(traj3B.end())
                .lineToLinearHeading(new Pose2d(37, -86, Math.toRadians(-91)),
                        SampleMecanumDrive.getVelocityConstraint(0.75 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj5B = drive.trajectoryBuilder(traj4A.end())
                .lineToLinearHeading(new Pose2d(38, -90, Math.toRadians(-91)),
                        SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj6B = drive.trajectoryBuilder(traj5B.end().plus(new Pose2d(0, -5, Math.toRadians(0))))
                .lineToLinearHeading(new Pose2d(37, 10, Math.toRadians(-91)),
                        SampleMecanumDrive.getVelocityConstraint(0.75 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj7B = drive.trajectoryBuilder(traj6B.end())
                .lineToLinearHeading(new Pose2d(37, 15, Math.toRadians(-91)),
                        SampleMecanumDrive.getVelocityConstraint(0.25 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addSpatialMarker(new Vector2d(37, 16), () -> {
                    spintake.spin(0.5, 3000);
                })
                .build();

        Trajectory traj8B = drive.trajectoryBuilder(traj7B.end())
                .lineToLinearHeading(new Pose2d(37, -88, Math.toRadians(-91)),
                        SampleMecanumDrive.getVelocityConstraint(0.75 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj9B = drive.trajectoryBuilder(traj8B.end())
                .lineToLinearHeading(new Pose2d(38, -90, Math.toRadians(-91)),
                        SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        /** -------------------------------------------------------------------------------------
                                                POS = 3
         ------------------------------------------------------------------------------------- */

        Trajectory traj1C = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(0)))
                .build();

        Trajectory traj2C = drive.trajectoryBuilder(traj1C.end())
                .lineToLinearHeading(new Pose2d(40, 8, Math.toRadians(90)))
                .build();

        Trajectory backwardsC = drive.trajectoryBuilder(traj2C.end())
                .lineToLinearHeading(new Pose2d(40, 10, Math.toRadians(90)))
                .build();

        Trajectory traj3C = drive.trajectoryBuilder(backwardsC.end())
                .lineToLinearHeading(new Pose2d(65, 12, Math.toRadians(91)))
                .build();

        Trajectory traj4C = drive.trajectoryBuilder(traj3C.end())
                .lineToLinearHeading(new Pose2d(65, 68, Math.toRadians(91)))
                .build();

        Trajectory traj5C = drive.trajectoryBuilder(traj4C.end())
                .lineToLinearHeading(new Pose2d(40, 80, Math.toRadians(91)),
                        SampleMecanumDrive.getVelocityConstraint(0.8 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj6C = drive.trajectoryBuilder(traj5C.end().plus(new Pose2d(0, -5, Math.toRadians(0))))
                .lineToLinearHeading(new Pose2d(8, 50, Math.toRadians(91)))
                .build();

        Trajectory traj7C = drive.trajectoryBuilder(traj6A.end())
                .lineToLinearHeading(new Pose2d(8, 0, Math.toRadians(91)))
                .build();

        Trajectory traj8C = drive.trajectoryBuilder(traj7C.end())
                .lineToLinearHeading(new Pose2d(37, -32, Math.toRadians(91)),
                        SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj9C = drive.trajectoryBuilder(traj8C.end())
                .lineToLinearHeading(new Pose2d(65, 0, Math.toRadians(91)))
                .build();

        Trajectory traj10C = drive.trajectoryBuilder(traj9C.end())
                .lineToLinearHeading(new Pose2d(65, 68, Math.toRadians(91)))
                .build();

        Trajectory traj11C = drive.trajectoryBuilder(traj10C.end())
                .lineToLinearHeading(new Pose2d(35, 80, Math.toRadians(91)))
                .build();

        /* Trajectory traj2A = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(90)))
                .build();

        Trajectory traj2C = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(35, 10, Math.toRadians(-90)))
                .build();

        Pose2d traj2End = traj1.end();
        if (pos == 1)
           traj2End = traj2A.end();
        if (pos == 3)
            traj2End = traj2C.end();

        Trajectory traj3B = drive.trajectoryBuilder(traj2End)
                .lineToLinearHeading(new Pose2d(60, 10, Math.toRadians(-90)))
                .build();

        Trajectory traj3A = drive.trajectoryBuilder(traj2End)
                .lineToLinearHeading(new Pose2d(60, -10, Math.toRadians(90)))
                .build();

        Pose2d traj3End = traj1.end();
        if (pos == 1)
            traj3End = traj3A.end();
        if (pos == 3)
            traj3End = traj3B.end();

        Trajectory traj4 = drive.trajectoryBuilder(traj3End)
                .lineToLinearHeading(new Pose2d(60, -75, Math.toRadians(-90)))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(40, -88, Math.toRadians(90)))
                .build();
         */

        waitForStart();

        drive.followTrajectory(traj1A);
        fourBar.closeClaw();

        if (pos == 1)
            drive.followTrajectory(traj2A);
        if (pos == 3)
            drive.followTrajectory(traj2C);

        spintake.outtake(0.4, 300);
        spintake.stop();
        sleep(200);
        if (pos == 2)
            drive.followTrajectory(backwardsB);
        sleep(400);

        /* if (pos == 2)
            drive.turn(Math.toRadians(90));
        if (pos == 3)
            drive.turn(Math.toRadians(180));
         */

        if (pos == 1) {
            drive.followTrajectory(traj3A);
            drive.followTrajectory(traj4A);
            sleep(1000);
            drive.followTrajectory(traj5A);
        }
        if (pos == 2) {
            drive.followTrajectory(traj3B);
            drive.followTrajectory(traj4B);
            sleep(1000);
            drive.followTrajectory(traj5B);
        }
        if (pos == 3) {
            drive.followTrajectory(backwardsC);
            drive.followTrajectory(traj3C);
            drive.followTrajectory(traj4C);
            sleep(1000);
            drive.followTrajectory(traj5C);
        }

        fourBar.closeClaw();
        fourBar.raiseFourBar();
        fourBar.closeClaw();
        sleep(700);
        slides.moveSlidesToHeightABS(80, 0.7);
        fourBar.lowerFourBar();
        sleep(700);
        sensorDistance.distanceDetection(hardwareMap,11.8);
        fourBar.openClaw();
        fourBar.raiseFourBar();
        slides.moveSlidesToHeightABS(35, 0.4);

        if (pos == 3) {
            drive.followTrajectory(traj6C);
            drive.followTrajectory(traj7C);
            drive.followTrajectory(traj8C);
        }

        if (pos == 1) {
            drive.followTrajectory(traj6A);
            drive.followTrajectory(traj7A);
            drive.followTrajectory(traj8A);
        }

        spintake.spin(0.5, 700);

        if (pos == 2) {
            drive.followTrajectory(traj6B);
            drive.followTrajectory(traj7B);
            spintake.stop();
            sleep(500);
            drive.followTrajectory(traj8B);
            drive.followTrajectory(traj9B);

        }

        if (pos == 1){
            drive.followTrajectory(traj9A);
            drive.followTrajectory(traj10A);
            drive.followTrajectory(traj11A);
        }

        if (pos == 3){
            drive.followTrajectory(traj9C);
            drive.followTrajectory(traj10C);
            drive.followTrajectory(traj11C);
        }

        /**fourBar.closeClaw();
         sleep(750);
         fourBar.raiseFourBar();
         sleep(500);
         slides.moveSlidesToHeightABS(1500, 0.7);
         sleep(300);
         fourBar.lowerFourBar();
         sleep(1000);
         fourBar.openClaw();
         }
         if (pos == 1) {
         drive.followTrajectory(traj2A);
         spintake.outtake(500);
         sleep(200);

         }
         if (pos == 3) {
         drive.followTrajectory(traj2C);
         spintake.outtake(500);
         sleep(200);

         drive.followTrajectory(traj3B);
         }

         drive.followTrajectory(traj4);
         fourBar.closeClaw();
         sleep(200);
         fourBar.rotatePos(1);
         slides.moveSlidesToHeightABS(2000, 0.75);
         fourBar.rotatePos(0);
         sleep(200);
         fourBar.openClaw();*/


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
