package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {

    private DcMotorEx slide;
    private LinearOpMode opmode;

    public Slides(HardwareMap hardwareMap, LinearOpMode opmode) {
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        // slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.opmode = opmode;
    }

    public void moveSlides() {
        double power = opmode.gamepad2.right_stick_y;
        // double pos = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2.0;

        double pos = slide.getCurrentPosition();

        if (!(pos > 2000 || pos < -50)) {
            slide.setPower(power);
            // slideRight.setPower(-power);
        } else {
            slide.setPower(0);
            // slideRight.setPower(0);
        }

        opmode.telemetry.addData("slideLeft:", slide.getCurrentPosition());
        // opmode.telemetry.addData("slideRight:", slideRight.getCurrentPosition());
        opmode.telemetry.update();
    }

    public void resetSlides() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
