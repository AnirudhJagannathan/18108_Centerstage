package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {

    private DcMotorEx slideLeft, slideRight;
    private LinearOpMode opmode;

    public Slides(HardwareMap hardwareMap, LinearOpMode opmode) {
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.opmode = opmode;
    }

    public void moveSlides() {
        double power = opmode.gamepad2.right_stick_y;
        double pos = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2.0;

        if (!(pos > 2000 || pos < -50)) {
            slideLeft.setPower(power);
            slideRight.setPower(-power);
        } else {
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }

        opmode.telemetry.addData("slideLeft:", slideLeft.getCurrentPosition());
        opmode.telemetry.addData("slideRight:", slideRight.getCurrentPosition());
        opmode.telemetry.update();
    }

    public void resetSlides() {
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
