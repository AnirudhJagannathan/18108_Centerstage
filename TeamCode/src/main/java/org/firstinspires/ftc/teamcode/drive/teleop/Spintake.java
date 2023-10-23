package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Spintake {

    private DcMotorEx spintake;
    private final double POWER = 0.6;

    public Spintake(HardwareMap hardwareMap) {
        spintake = hardwareMap.get(DcMotorEx.class, "spintake");
        spintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void spin(boolean bPressed, boolean xPressed) {
        if (bPressed)
            spintake.setPower(POWER);
        if (xPressed)
            spintake.setPower(0);
    }
}
