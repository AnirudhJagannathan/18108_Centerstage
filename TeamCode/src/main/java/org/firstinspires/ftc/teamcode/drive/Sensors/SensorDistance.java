package org.firstinspires.ftc.teamcode.drive.Sensors;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class SensorDistance {
    private DistanceSensor distance;
    private LinearOpMode opmode;
    public SensorDistance(HardwareMap hardwareMap, LinearOpMode opMode) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        distance = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        this.opmode = opMode;
    }

    public double lengthDetection() {
        opmode.telemetry.addData("Distance", distance.getDistance(DistanceUnit.INCH));
        opmode.telemetry.update();
        return distance.getDistance(DistanceUnit.INCH);
    }
    public void distanceDetection(HardwareMap hardwareMap){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        while (lengthDetection() > 8)
            drive.setMotorPowers(-0.4, -0.4, -0.4, -0.4);
        drive.setMotorPowers(0,0,0,0);
        opmode.telemetry.addData("LineDetection", "We have parked");
    }
}
