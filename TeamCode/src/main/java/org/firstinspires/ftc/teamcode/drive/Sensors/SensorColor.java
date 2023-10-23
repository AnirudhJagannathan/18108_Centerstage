package org.firstinspires.ftc.teamcode.drive.Sensors;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class SensorColor {
    private ColorSensor color;
    private LinearOpMode opmode;
    private DcMotorEx frontLeft, frontRight, backRight, backLeft;
    public SensorColor(HardwareMap hardwareMap, LinearOpMode opMode) {
        color = hardwareMap.get(ColorSensor.class, "colorSensor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        this.opmode = opMode;
    }
    public void colorDetection(boolean redOrBlue) {

        float[] hsvValues = {0F, 0F, 0F};
        color.enableLed(false);
        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);
        opmode.telemetry.addData("Red", hsvValues[0]);
        opmode.telemetry.addData("Blue", hsvValues[2]);
        if (redOrBlue && hsvValues[0] > 200) {
            opmode.telemetry.addData("Blue", "You are over Blue");
            opmode.telemetry.update();
        } else if (!redOrBlue && hsvValues[0] < 35) {
            opmode.telemetry.addData("Red", "You are over Red");
            opmode.telemetry.update();
        }
        else{
            frontRight.setPower(0.5);
            frontLeft.setPower(0.5);
            backRight.setPower(0.5);
            backLeft.setPower(0.5);
        }
        opmode.telemetry.update();
    }
}
