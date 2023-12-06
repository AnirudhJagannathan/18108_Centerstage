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
            slides.resetSlides();
            launcher.resetPos();
            fourBar.closeClaw();
        }

        waitForStart();

        while (!isStopRequested()) {
            boolean spintakeOn = false;
              /* drive.setWeightedDrivePower(n
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
              );
               */

            csBot.mecanumDriving();

            if (slides.getCurrentPos() > 120) {
                if (gamepad2.left_bumper)
                    spintake.spin();
                spintakeOn = true;
            }

            if (slides.getCurrentPos() > 25 && slides.getCurrentPos() < 150)
                fourBar.raiseFourBar();
            else
                fourBar.manualControl(spintakeOn);

            if (slides.getCurrentPos() < 120) {
                // slides.moveSlides(true);
                while (spintake.getPixelBarPos() < 0.9) {
                    spintake.raiseBar();
                }
                spintakeOn = false;
            }
            if (gamepad2.x)
                spintake.stop();
            if (gamepad2.right_bumper)
                spintake.outtake();

            slides.moveSlides(false);

            if (gamepad1.a)
                slides.resetSlides();
            if (gamepad1.y)
                launcher.launch();
            if (gamepad1.b)
                launcher.resetPos();
            if (gamepad2.dpad_up)
                fourBar.rotate();
            if (gamepad1.left_trigger > 0.1)
                hanging.lift();
            if (gamepad1.right_trigger > 0.1)
                hanging.lower();
            if (gamepad2.back)
                sensorDistance.distanceDetection(hardwareMap, 4);
            if (gamepad2.a)
                fourBar.closeClaw();
            if (gamepad2.b)
                fourBar.openClaw();
        }
    }

    public void depositClaw() {
        fourBar.rotatePos(0.0);
        sleep(300);
        sensorDistance.distanceDetection(hardwareMap,4);
        fourBar.openClaw();
        sleep(300);
        fourBar.rotatePos(1.0);
        sleep(300);
        //slides.moveSlidesAuto(0);
        sleep(300);
        fourBar.rotatePos(0.8);
    }


    public void collectPixel(){
        spintake.raiseBar();
        slides.moveSlidesToHeightABS(10, 0.4);
        sleep(300);
        fourBar.rotatePos(0.8);
        sleep(300);
        fourBar.closeClaw();
        sleep(300);
        fourBar.rotatePos(1.0);
    }
}
