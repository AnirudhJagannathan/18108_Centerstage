package org.firstinspires.ftc.teamcode.drive.teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class Launcher {
    private Servo launcher;
    private LinearOpMode opmode;
    private final double START_POS = 0.0;
    private final double END_POS = 0.4;

    public Launcher(HardwareMap hardwareMap, LinearOpMode opmode) {
        launcher = hardwareMap.get(Servo.class, "launcher");
        this.opmode = opmode;
        launcher.setPosition(START_POS);
    }

    public void resetPos() {
        if (opmode.gamepad2.left_bumper) {
            launcher.setPosition(START_POS);
        }
    }

    public void launch() {
        if (opmode.gamepad2.right_bumper)
            launcher.setPosition(END_POS);
    }


}
