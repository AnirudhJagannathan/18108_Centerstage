package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Sensors.SensorColor;

@TeleOp
public class TeleOp_Drive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        Spintake spintake = new Spintake(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        FourBar fourBar = new FourBar(hardwareMap);
        SensorColor sensorColor = new SensorColor(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );




            launcher.resetPos();

            double power = gamepad2.right_stick_y;
            slides.moveSlides(power);

            if (gamepad2.b)
                spintake.spin(true, false);
            if (gamepad2.x)
                spintake.spin(false, true);

            if (gamepad2.left_bumper)
                launcher.resetPos();
            if (gamepad2.right_bumper)
                launcher.launch();

            if (gamepad2.y)
                fourBar.resetPos();
            if (gamepad2.a)
                fourBar.rotate();
        }


        /* while (!isStopRequested()) {
            /* drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );


            double drive1 = gamepad1.left_stick_y;
            double strafe = gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_x;
            double v1, v2, v3, v4;

            if (gamepad1.right_bumper) {
                v1 = Range.clip(-drive1 + strafe + turn, -0.2, 0.2);
                v2 = Range.clip(-drive1 - strafe - turn, -0.2, 0.2);
                v3 = Range.clip(-drive1 + strafe - turn, -0.2, 0.2);
                v4 = Range.clip(-drive1 - strafe + turn, -0.2, 0.2);
            }

            else {
                v1 = Range.clip(-drive1 + strafe + turn, -0.85, 0.85);
                v2 = Range.clip(-drive1 - strafe - turn, -0.85, 0.85);
                v3 = Range.clip(-drive1 + strafe - turn, -0.85, 0.85);
                v4 = Range.clip(-drive1 - strafe + turn, -0.85, 0.85);
            }
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);

        }

         */
    }

    // Anya Test commit

}
