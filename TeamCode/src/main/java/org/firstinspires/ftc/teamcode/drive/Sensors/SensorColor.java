package org.firstinspires.ftc.teamcode.drive.Sensors;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class SensorColor {
    private ColorSensor color;
    public SensorColor(HardwareMap hardwareMap) {
        color = hardwareMap.get(ColorSensor.class, "colorSensor");
    }
    public void colorDetection(boolean redOrBlue) {

        float[] hsvValues = {0F, 0F, 0F};
        color.enableLed(false);
        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);
        Log.v("Red", String.valueOf(hsvValues[0]));
        Log.v("Blue", String.valueOf(hsvValues[2]));
        if (redOrBlue && hsvValues[0] > 200) {
            Log.v("Status: ", "You are over Blue");
        } else if (!redOrBlue && hsvValues[0] < 35) {
            Log.v("Red", "You are over Red");
        }
    }
}
