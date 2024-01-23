package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Spintake {

    private DcMotorEx spintake;
    private Servo pixelBar;
    private Servo pixelStick1;
    private Servo pixelStick2;
    private LinearOpMode opmode;
    // private Slides slides;
    private final double POWER = 1.0;
    private final double BAR_START = 1;
    private final double BAR_END = 0.3;
    private final double STICK_START = 0.15;
    private final double STICK_MID = 0.475;
    private final double STICK_END = 0.8;

    public Spintake(HardwareMap hardwareMap, LinearOpMode opmode) {
        spintake = hardwareMap.get(DcMotorEx.class, "spintake");
        pixelBar = hardwareMap.get(Servo.class, "pixelBar");
        pixelStick1 = hardwareMap.get(Servo.class, "pixelStick1");
        pixelStick2 = hardwareMap.get(Servo.class, "pixelStick2");
        spintake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spintake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.opmode = opmode;
    }

    public void spin() {
        pixelBar.setPosition(BAR_END);
        spintake.setPower(POWER);
        stickOut();
    }
    public void stop() {
        spintake.setPower(0);
    }

    public void outtake() {
        spintake.setPower(-POWER);
        stickOut();
    }

    public void spin(double power, long time) {
        spintake.setPower(power);
        // opmode.sleep(time);
    }

    public void outtake(double power, long time) {
        spintake.setPower(-power);
        opmode.sleep(time);
    }

    public void raiseBar() {
        pixelBar.setPosition(BAR_START);
    }

    public void lowerBar() {
        pixelBar.setPosition(BAR_END);
    }

    public void stickIn() {
        pixelStick1.setPosition(1 - STICK_END);
        pixelStick2.setPosition(STICK_END);
    }

    public void stickOut() {
        pixelStick1.setPosition(1 - STICK_START);
        pixelStick2.setPosition(STICK_START);
    }

    public void stickIntake() {
        pixelStick1.setPosition(1 - STICK_END);
        while (pixelStick1.getPosition() < 1 - STICK_END) {

        }
        opmode.sleep(300);
        pixelStick1.setPosition(1 - STICK_MID);

        pixelStick2.setPosition(STICK_END);
        while (pixelStick2.getPosition() > STICK_END) {

        }
        opmode.sleep(300);
        pixelStick2.setPosition(STICK_MID);
    }

    public double getPixelBarPos() {
        return pixelBar.getPosition();
    }

    public Servo getPixelBar() {
        return pixelBar;
    }

    public Servo getPixelStick1() {
        return pixelStick1;
    }

    public Servo getPixelStick2() {
        return pixelStick2;
    }
}