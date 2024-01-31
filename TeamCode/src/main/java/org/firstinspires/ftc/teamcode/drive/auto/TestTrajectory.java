package org.firstinspires.ftc.teamcode.drive.auto;
//anirudh is incredibly stupid and unintelligent

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.Vision.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.dataFromOpenCV;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Sensors.SensorDistance;
import org.firstinspires.ftc.teamcode.drive.teleop.FourBar;
import org.firstinspires.ftc.teamcode.drive.teleop.Slides;
import org.firstinspires.ftc.teamcode.drive.teleop.Spintake;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous
public class TestTrajectory extends LinearOpMode {
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
    PixelDetectionPipeline pipeline = new PixelDetectionPipeline();
    private SensorDistance distance;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag IDs from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    enum trajectory{
        traj1,
        traj2,
        traj3,
        done
    }

    trajectory traj = trajectory.traj1;

    AprilTagDetection tagOfInterest = null;

    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        spintake = new Spintake(hardwareMap, this);
        // SensorDistance sensorDistance = new SensorDistance(hardwareMap, this);
        slides = new Slides(hardwareMap, this);
        fourBar = new FourBar(hardwareMap, this);
        webcam1 = new Webcam(hardwareMap, "Webcam 1");
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        distance = new SensorDistance(hardwareMap, this);
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
        webcam1.setPipeline(pipeline);

        // webcam1.getWebcam().setPipeline(aprilTagDetectionPipeline);
        //Creating Autonomous trajectory


        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        // spintake.stickOut();

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(0)))
                /**.addDisplacementMarker(() -> {
                    telemetry.addData("position:", pipeline.position);
                    telemetry.addData("vals:", Arrays.toString(pipeline.vals));
                    telemetry.update();
                })*/
                .build();

        Trajectory traj2 = drive.trajectoryBuilder((traj1.end()))
                .lineToLinearHeading(new Pose2d(25, 25, Math.toRadians(0)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(1, 1, Math.toRadians(90)))
                .build();

        traj = trajectory.traj1;

        waitForStart();

        while (opModeIsActive()){
            switch (traj) {
                case traj1:
                    drive.followTrajectoryAsync(traj1);
                    if (!drive.isBusy()){
                        traj = trajectory.traj2;
                    }
                case traj2:
                    drive.followTrajectoryAsync(traj2);
                    if (!drive.isBusy()){
                        traj = trajectory.traj3;
                    }
                case traj3:
                    drive.followTrajectoryAsync(traj3);
                    if (!drive.isBusy()){
                        traj = trajectory.done;
                    }
                case done:
                    break;
            }
            if (distance.lengthDetection() <= 11.8){
                traj = trajectory.done;
            }
            drive.update();
        }

        webcam1.getWebcam().stopStreaming();
    }

}
