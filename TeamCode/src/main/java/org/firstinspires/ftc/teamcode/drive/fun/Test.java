package org.firstinspires.ftc.teamcode.drive.fun;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Sensors.SensorDistance;


@Autonomous
public class Test extends LinearOpMode {
    private Servo claw;
    private double position;
    private SensorDistance distance;

    @Override
    public void runOpMode() throws InterruptedException{
        claw = hardwareMap.get(Servo.class, "claw");
        distance = new SensorDistance(hardwareMap, this);
        position = 0;
        waitForStart();

        while (opModeIsActive()){
            /**if (gamepad2.b)
                position = 0;
            if (gamepad2.a)
                position = 1;
            claw.setPosition(position);*/
            telemetry.addData("distance", distance.MRdistanceSensor());
            telemetry.update();
        }
    }
}
