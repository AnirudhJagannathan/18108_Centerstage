package org.firstinspires.ftc.teamcode.drive.teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class Launcher {
    private Servo launcher;
    private final double START_POS = 0.0;
    private final double END_POS = 0.25;

    public Launcher(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(Servo.class, "launcher");
    }

    public void resetPos() {
        launcher.setPosition(START_POS);
    }

    public void launch() {
        launcher.setPosition(END_POS);
    }
}
