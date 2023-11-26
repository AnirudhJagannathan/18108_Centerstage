package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Spintake {

    private DcMotorEx spintake;
    private LinearOpMode opmode;
    private boolean on;
    private final double POWER = 0.25;

    public Spintake(HardwareMap hardwareMap, LinearOpMode opmode) {
        spintake = hardwareMap.get(DcMotorEx.class, "spintake");
        spintake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.opmode = opmode;
        on = false;
    }


    public void spin() {
        if (opmode.gamepad2.left_bumper) {
            spintake.setPower(POWER);
            // on = true;
        } else if (opmode.gamepad2.x) {
            spintake.setPower(0);
            // on = false;
        }
        // when pixel gets stuck
        if (spintake.getCurrent(CurrentUnit.AMPS) > 4.6){
            spintake.setPower(POWER);
            opmode.sleep(500);
            spintake.setPower(0);
            opmode.sleep(500);
            spintake.setPower(-POWER * 0.3);
            opmode.sleep(750);
            spintake.setPower(POWER);
        }
        opmode.telemetry.addData("Spintake", spintake.getCurrentPosition());
        opmode.telemetry.addData("Current Drawn", spintake.getCurrent(CurrentUnit.AMPS));
    }

    public void outtake() {
        if (opmode.gamepad2.right_bumper)
            spintake.setPower(-POWER);
    }

    public void outtake(long time) {
        spintake.setPower(-0.5 * POWER);
        opmode.sleep(time);
    }
}