package org.firstinspires.ftc.teamcode.drive.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class MasterAuto extends LinearOpMode {
    BlueLeft blueLeft = new BlueLeft();
    BlueRight blueRight = new BlueRight();
    RedLeft redLeft = new RedLeft();
    RedRight redRight = new RedRight();

    public void runOpMode() throws InterruptedException {
        short code = -1;
        while (opModeInInit()){
            if (gamepad1.x)
                code = 0;

            else if (gamepad1.y)
                code = 1;

            else if (gamepad1.a)
                code = 2;

            else if (gamepad1.b)
                code = 3;
        }

        waitForStart();

        while (opModeIsActive())
            if (code == 0) {
                //blueRight.BlueRightAuto();
                telemetry.addData("BlueRight", "BlueRight program is playing now");
                telemetry.update();
            }
            else if (code == 1) {
                //blueLeft.BlueLeftAuto();
                telemetry.addData("BlueLeft", "BlueLeft program is playing now");
                telemetry.update();
            }
            else if (code == 2) {
                //redLeft.RedLeftAuto();
                telemetry.addData("RedLeft", "RedLeft program is playing now");
                telemetry.update();
            }
            else if (code == 3) {
                //redRight.RedRightAuto();
                telemetry.addData("RedRight", "RedRight program is playing now");
                telemetry.update();
            }
            else {
                telemetry.addData("No Auto", "Your stupid eaglederp, stop being a blud");
                telemetry.update();
            }
    }
}
