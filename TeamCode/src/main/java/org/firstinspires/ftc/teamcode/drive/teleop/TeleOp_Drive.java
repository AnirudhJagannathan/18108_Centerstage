package org.firstinspires.ftc.teamcode.drive.teleop;

import android.hardware.Sensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Sensors.SensorDistance;
import org.firstinspires.ftc.teamcode.drive.Sensors.SensorDistance;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOp_Drive extends LinearOpMode {

    private SampleMecanumDrive drive;
    private CenterstageBot csBot;
    private Launcher launcher;
    private Spintake spintake;
    private Slides slides;
    private FourBar fourBar;
    private Hanging hanging;
    private SensorDistance sensorDistance;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        csBot = new CenterstageBot(hardwareMap, this);
        launcher = new Launcher(hardwareMap, this);
        spintake = new Spintake(hardwareMap, this);
        slides = new Slides(hardwareMap, this);
        fourBar = new FourBar(hardwareMap, this);
        hanging = new Hanging(hardwareMap, this);
        sensorDistance = new SensorDistance(hardwareMap, this);

        // SensorDistance sensorDistance = new SensorDistance(hardwareMap, this);

        /*drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.resetSlides();*/
        while (opModeInInit()){
            launcher.resetPos();
            fourBar.closeClaw();
        }

        waitForStart();

        while (!isStopRequested()) {
              /* drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
              );
               */


            csBot.mecanumDriving();


            spintake.spin();
            spintake.outtake();
            //fourBar.manualControl();
            slides.moveSlides();
            /* if (gamepad1.x) {
                sensorDistance.lengthDetection();
                sensorDistance.distanceDetection(hardwareMap);
                }
            */

            if (gamepad2.y)
                launcher.launch();
            if (gamepad1.b)
                launcher.resetPos();
           if (gamepad2.dpad_up)
                fourBar.rotate();
            if (gamepad1.left_trigger > 0.1)
                hanging.lift();
            if (gamepad1.right_trigger > 0.1)
                hanging.lower();
            if (gamepad2.start)
                collectPixel();
            if (gamepad2.back)
                depositClaw();
            if (gamepad2.a)
                fourBar.closeClaw();
            if (gamepad2.b)
                fourBar.openClaw();
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

    public void depositClaw(){
        fourBar.rotatePos(0.0);
        fourBar.openClaw();
        drive.setMotorPowers(1, 1,1, 1);
        sleep(300);
        slides.moveSlidesAuto(0);
        fourBar.rotatePos(0.8);
    }

    public void collectPixel() {
        fourBar.rotatePos(0.8);
        slides.moveSlidesAuto(-40);
        sleep(300);
        fourBar.closeClaw();
        sleep(300);
        slides.moveSlidesAuto(-20);
        fourBar.rotatePos(1.0);
    }

    // Anya Test commit

}
