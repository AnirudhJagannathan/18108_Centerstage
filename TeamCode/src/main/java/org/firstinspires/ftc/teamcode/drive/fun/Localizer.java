package org.firstinspires.ftc.teamcode.drive.fun;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


public class Localizer {
    private DcMotorEx rightEncoder, leftEncoder, frontEncoder;
    private DcMotorEx frontLeft, frontRight, backRight, backLeft;
    private LinearOpMode opmode;

    public Localizer(HardwareMap hardwareMap, LinearOpMode opMode) {
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rightEncoder = frontRight;
        leftEncoder = frontLeft;
        frontEncoder = backLeft;
        int[] position = {0, 0, 0};
        this.opmode = opMode;
    }
    public void position_tracking(){
        // tracking x-coordinate
        float x = rightEncoder.getCurrentPosition() + leftEncoder.getCurrentPosition();
    }
}
