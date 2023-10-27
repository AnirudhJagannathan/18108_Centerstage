package org.firstinspires.ftc.teamcode.drive.teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class Launcher {
    private Servo launcher;
    private LinearOpMode opmode;
    private final double START_POS = 1;
    private final double END_POS = 0.6;

    public Launcher(HardwareMap hardwareMap, LinearOpMode opmode) {
        launcher = hardwareMap.get(Servo.class, "launcher");
        this.opmode = opmode;
        launcher.setPosition(START_POS);
    }

    public void resetPos() {
        launcher.setPosition(START_POS);
        opmode.telemetry.addData("LauncherPos: ", launcher.getPosition());
        opmode.telemetry.update();
    }

    public void launch() {
        launcher.setPosition(END_POS);
        opmode.telemetry.addData("LauncherPos: ", launcher.getPosition());
        opmode.telemetry.update();
    }


}
