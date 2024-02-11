package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
//tanay you're a monkey
public class TankDrive {
    private LinearOpMode opmode;
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    public TankDrive(HardwareMap hardwareMap, LinearOpMode opmode) {
        this.opmode = opmode;
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
    }
    public void tankDriving() {
        double leftDrive = -opmode.gamepad1.left_stick_y;
        double rightDrive = -opmode.gamepad1.right_stick_y;
        double v0, v1, v2, v3;

        if (opmode.gamepad1.a) {
            v0 = 0.75;
            v1 = 0.75;
            v2 = 0.75;
            v3 = 0.75;
        } else if (opmode.gamepad1.b) {
            v0 = -0.2;
            v1 = -0.2;
            v2 = -0.2;
            v3 = -0.2;
        } else if (opmode.gamepad1.left_bumper) {
            v0 = -0.75;
            v1 = 0.75;
            v2 = 0.75;
            v3 = -0.75;
        } else if (opmode.gamepad1.right_bumper) {
            v0 = 0.75;
            v1 = -0.75;
            v2 = -0.75;
            v3 = 0.75;
        } else {
            v0 = Range.clip(leftDrive, -1, 1);
            v1 = Range.clip(rightDrive, -1, 1);
            v2 = Range.clip(leftDrive, -1, 1);
            v3 = Range.clip(rightDrive, -1, 1);
        }
        leftFront.setPower(v0);
        rightFront.setPower(v1);
        leftBack.setPower(v2);
        rightBack.setPower(-v3);
    }
}
