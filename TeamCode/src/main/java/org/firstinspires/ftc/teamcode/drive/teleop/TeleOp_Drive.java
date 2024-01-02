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
    private TankDrive tankDrive;
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
        tankDrive = new TankDrive(hardwareMap, this);
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
            slides.resetSlides();
            launcher.resetPos();
            fourBar.closeClaw();
            spintake.stickOut();
        }

        waitForStart();

        while (!isStopRequested()) {
            csBot.mecanumDriving();

            if (gamepad2.left_bumper)
                spintake.spin();

            if (slides.getCurrentPos() < 150) {
                while (spintake.getPixelBarPos() < 0.9) {
                    spintake.raiseBar();
                }
            }
            if (gamepad2.right_bumper)
                spintake.outtake();
            if (gamepad2.x) {
                spintake.stop();
                spintake.stickIn();
            }

            slides.moveSlides(false);

            if (gamepad1.x)
                slides.resetSlides();
            if (gamepad1.y)
                launcher.launch();
            if (gamepad1.b)
                launcher.resetPos();
            if (gamepad1.left_trigger > 0.1)
                hanging.lift();
            if (gamepad1.right_trigger > 0.1)
                hanging.lower();
            if (gamepad1.left_bumper)
                hanging.smallerLift();
            if (gamepad2.a)
                fourBar.closeClaw();
            if (gamepad2.b)
                fourBar.openClaw();

            if (gamepad2.dpad_down)
                fourBar.lowerFourBar();
            if (gamepad2.dpad_up)
                fourBar.raiseFourBar();
            if (gamepad2.dpad_left)
                spintake.raiseBar();
            if (gamepad2.dpad_right)
                spintake.lowerBar();


            if (gamepad2.left_trigger > 0.1) {
                spintake.stickIn();
            }
            if (gamepad2.right_trigger > 0.1) {
                spintake.stickOut();
            }

            if (gamepad2.start) {
                spintake.stickIntake();
                sleep(250);
            }

        }
    }

    public void depositPixel() {
        fourBar.rotatePos(0.0);
        sleep(300);
        sensorDistance.distanceDetection(hardwareMap,2);
        fourBar.openClaw();
        sleep(300);
        fourBar.rotatePos(1.0);
        sleep(300);
        //slides.moveSlidesAuto(0);
        sleep(300);
        fourBar.rotatePos(0.8);
    }


    public void collectPixel() throws InterruptedException {
        fourBar.openClaw();
        fourBar.raiseFourBar();
        sleep(300);
        slides.moveSlidesToHeightABS(25, 0.5);
        sleep(300);
        fourBar.lowerFourBar();
        sleep(500);
        fourBar.closeClaw();
    }
}
