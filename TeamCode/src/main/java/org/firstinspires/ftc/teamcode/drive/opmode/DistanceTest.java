package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.CameraFusedPID;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class DistanceTest extends LinearOpMode {

    private OpenCvCamera controlHubCam;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        waitForStart();

        double width = CameraFusedPID.getWidth();
        double distance = CameraFusedPID.getDistance(width);

        telemetry.addData("Distance:", distance);
        telemetry.update();
        sleep(5000);

        while (distance > 5) {
            telemetry.addData("Distance:", distance);
            telemetry.update();
            drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
            width = CameraFusedPID.getWidth();
            distance = CameraFusedPID.getDistance(width);
        }

        drive.setMotorPowers(0, 0, 0, 0);

    }

}
