package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Spintake {

    private DcMotorEx spintake;
    private LinearOpMode opmode;
    private final double POWER = 0.6;

    public Spintake(HardwareMap hardwareMap, LinearOpMode opmode) {
        spintake = hardwareMap.get(DcMotorEx.class, "spintake");
        this.opmode = opmode;
    }

    public void spin() {
        if (opmode.gamepad2.b)
            spintake.setPower(POWER);
        if (opmode.gamepad2.x)
            spintake.setPower(0);
    }
}
