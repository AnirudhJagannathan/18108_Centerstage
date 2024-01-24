package org.firstinspires.ftc.teamcode.drive.Sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class SensorColor {
    private ColorSensor color;
    private LinearOpMode opMode;
    public SensorColor(HardwareMap hardwareMap, LinearOpMode opMode) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        color = hardwareMap.get(ColorSensor.class, "colorSensor");

    }

    public float[] colorDetection() {
        float[] hsvValue = {0F, 0F, 0F};
        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValue);
        return hsvValue;
    }

    public void pixelDetection(float red, float green, float blue){
        if (colorDetection()[0] > red || colorDetection()[1] > green || colorDetection()[2] > blue){
            opMode.telemetry.addData("Pixel Detected", colorDetection());
        }
        else{
            opMode.telemetry.addData("No Pixel", colorDetection());
        }
    }
}
