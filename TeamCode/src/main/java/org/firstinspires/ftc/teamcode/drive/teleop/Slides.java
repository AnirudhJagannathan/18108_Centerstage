package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {

    private DcMotorEx slideLeft, slideRight;
    private LinearOpMode opmode;

    public Slides(HardwareMap hardwareMap, LinearOpMode opmode) {
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        this.opmode = opmode;
    }

    public void moveSlides() {
        double power = opmode.gamepad2.right_stick_y;

        slideLeft.setPower(power);
        slideRight.setPower(-power);
    }

}
