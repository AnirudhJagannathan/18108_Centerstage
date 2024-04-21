package org.firstinspires.ftc.teamcode.drive.Sensors;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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
    private ModernRoboticsI2cRangeSensor Distance;
    private LinearOpMode opmode;
    public SensorDistance(HardwareMap hardwareMap, LinearOpMode opMode) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Distance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceSensor");
        this.opmode = opMode;
    }

    public double lengthDetection() {
        opmode.telemetry.addData("Raw Optical", Distance.rawOptical());
        opmode.telemetry.addData("Raw Ultrasonic", Distance.rawUltrasonic());
        opmode.telemetry.addData("CM Optical", Distance.cmOptical());
        opmode.telemetry.addData("CM Ultrasonic", Distance.cmUltrasonic());
        opmode.telemetry.addData("Distance", Distance.getDistance(DistanceUnit.INCH));
        opmode.telemetry.update();
        return distance.getDistance(DistanceUnit.INCH);
    }

    public double MRdistanceSensor() {
        opmode.telemetry.addData("Raw Optical", Distance.rawOptical());
        opmode.telemetry.addData("Raw Ultrasonic", Distance.rawUltrasonic());
        opmode.telemetry.addData("CM Optical", Distance.cmOptical());
        opmode.telemetry.addData("CM Ultrasonic", Distance.cmUltrasonic());
        opmode.telemetry.addData("Distance", Distance.getDistance(DistanceUnit.INCH));
        opmode.telemetry.update();
        return Distance.getDistance(DistanceUnit.INCH);
    }
    
    public void distanceDetection(HardwareMap hardwareMap, double distance){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        while (true) {
            if (lengthDetection() > distance) {
                drive.setMotorPowers(-0.4, -0.4, -0.4, -0.4);
            } else if (lengthDetection() <= distance) {
                drive.setMotorPowers(0,0,0,0);
                break;
            }
        }
        opmode.telemetry.addData("LineDetection", "We have parked");
    }
}
