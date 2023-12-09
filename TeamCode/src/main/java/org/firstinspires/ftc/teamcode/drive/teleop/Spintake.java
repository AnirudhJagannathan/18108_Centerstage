package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Spintake {

    private DcMotorEx spintake;
    private Servo pixelBar;
    private LinearOpMode opmode;
    // private Slides slides;
    private final double POWER = 0.8;
    private final double BAR_START = 1;
    private final double BAR_END = 0.3;

    public Spintake(HardwareMap hardwareMap, LinearOpMode opmode) {
        spintake = hardwareMap.get(DcMotorEx.class, "spintake");
        pixelBar = hardwareMap.get(Servo.class, "pixelBar");
        spintake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spintake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pixelBar.setPosition(BAR_START);
        this.opmode = opmode;

        // slides = new Slides(hardwareMap, this.opmode);
    }

    public void spin() {
        spintake.setPower(POWER);
        pixelBar.setPosition(BAR_END);
        // when pixel gets stuck
        if (spintake.getCurrent(CurrentUnit.AMPS) > 4.6){
            spintake.setPower(0);
            opmode.sleep(250);
            spintake.setPower(-POWER * 1);
        }
        opmode.telemetry.addData("Spintake", spintake.getCurrentPosition());
        opmode.telemetry.addData("Current Drawn", spintake.getCurrent(CurrentUnit.AMPS));
    }
    public void stop() {
        spintake.setPower(0);
    }

    public void outtake() {
        spintake.setPower(-POWER);
    }

    public void spin(long time) {
        spintake.setPower(POWER);
        // opmode.sleep(time);
    }

    public void outtake(long time) {
        spintake.setPower(-0.4 * POWER);
        opmode.sleep(time);
    }

    public void raiseBar() {
        pixelBar.setPosition(BAR_START);
    }

    public double getPixelBarPos() {
        return pixelBar.getPosition();
    }
}