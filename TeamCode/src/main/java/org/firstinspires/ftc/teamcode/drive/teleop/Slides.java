package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {

    private DcMotorEx slideLeft;
    private DcMotorEx slideRight;
    private DcMotorEx HSlides;
    private LinearOpMode opmode;
    // private Spintake spintake;

    public Slides(HardwareMap hardwareMap, LinearOpMode opmode) {
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        HSlides = hardwareMap.get(DcMotorEx.class, "spintake");

        // slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.opmode = opmode;

        // spintake = new Spintake(hardwareMap, this.opmode);
    }

    public void moveSlides(boolean belowLim) {
        double power = opmode.gamepad2.right_stick_y;
        if (belowLim)
            power = 0.15;
        double pos = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2.0;
        opmode.telemetry.addData("slides: ", pos);
        opmode.telemetry.update();
        /* if (getCurrentPos() < 700 && spintake.getPixelBarPos() < 0.3) {
            spintake.raiseBar();
        }

         */

        if (!((pos > 370 && power < 0)  || (pos < -50 && power > 0))) {
            slideLeft.setPower(power);
            slideRight.setPower(-power);
            opmode.telemetry.addData("powerL:", slideLeft.getCurrentPosition());
            opmode.telemetry.addData("powerR:", slideRight.getCurrentPosition());
            opmode.telemetry.update();
        } else {
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }
    }

    public void moveSlidesToHeightABS(int encoderPos, double power) {
        while (slideLeft.getCurrentPosition() > -encoderPos || slideLeft.getCurrentPosition() < -encoderPos) {
            if (slideLeft.getCurrentPosition() < -encoderPos) {
                slideLeft.setPower(power);
                slideRight.setPower(-power);
            } else {
                slideLeft.setPower(-power);
                slideRight.setPower(power);
            }
        }

        slideLeft.setPower(0);
        slideRight.setPower(0);
    }

    public void stop() {
        slideLeft.setPower(0);
        slideRight.setPower(0);
    }

    public void horizontalSlides() {
        double power = opmode.gamepad2.right_stick_y*0.5;
        double pos = HSlides.getCurrentPosition();
        if (!((pos > 370 && power < 0) || (pos < -50 && power > 0))) {
            HSlides.setPower(power);
        } else {
            HSlides.setPower(0);;
        }
    }

    public void resetSlides() {
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getCurrentPos() {
        return ((double) slideRight.getCurrentPosition() + slideLeft.getCurrentPosition());
    }
}