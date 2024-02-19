package org.firstinspires.ftc.teamcode.drive.teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class Launcher {
    private Servo launcher;
    private Servo placeHolder;
    private LinearOpMode opmode;
    private final double LAUNCHER_START_POS = 0.485;
    private final double LAUNCHER_END_POS = 0.800;
    private final double PLACEHOLDER_HOLD = 0.05;
    private final double PLACEHOLDER_RELEASE = 0.4;



    public Launcher(HardwareMap hardwareMap, LinearOpMode opmode) {
        launcher = hardwareMap.get(Servo.class, "launcher");
        placeHolder = hardwareMap.get(Servo.class, "placeHolder");
        this.opmode = opmode;
        launcher.setPosition(LAUNCHER_START_POS);
    }

    public void resetPos() {
        // placeHolder.setPosition(PLACEHOLDER_HOLD);
        // opmode.sleep(1000);
        launcher.setPosition(LAUNCHER_START_POS);
        opmode.telemetry.addData("LauncherPos: ", placeHolder.getPosition());
        opmode.telemetry.update();

        ((PwmControl) placeHolder).setPwmDisable();
    }

    public void launch() {
        // placeHolder.setPosition(PLACEHOLDER_RELEASE);
        // opmode.sleep(1000);
        launcher.setPosition(LAUNCHER_END_POS);
        opmode.telemetry.addData("LauncherPos: ", placeHolder.getPosition());
        opmode.telemetry.update();
    }

    public void trayStickIn() {
        placeHolder.setPosition(PLACEHOLDER_HOLD);
    }

    public void trayStickOut() {
        placeHolder.setPosition(PLACEHOLDER_RELEASE);
    }

    public Servo getLauncher() {
        return launcher;
    }

    public Servo getPlaceHolder() {
        return placeHolder;
    }
}
