package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Sensors.SensorDistance;

@Disabled
@TeleOp
public class TeleOp_DriveArvind extends LinearOpMode {

    private SampleMecanumDrive drive;
    private CenterstageBot csBot;
    private Launcher launcher;
    private Spintake spintake;
    private Slides slides;
    private FourBar fourBar;
    private Hanging hanging;
    private SensorDistance sensorDistance;
    private boolean distance;

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
            distance = true;
        }

        waitForStart();

        while (opModeIsActive()) {
            /*boolean spintakeOn = false;
              /* drive.setWeightedDrivePower(n
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
              );




            if (slides.getCurrentPos() > 120) {
                if (gamepad2.left_bumper)
                    spintake.spin();
                spintakeOn = true;
            }

            if (slides.getCurrentPos() > 25 && slides.getCurrentPos() < 150)
                fourBar.raiseFourBar();
            else
                //fourBar.manualControl(spintakeOn);

                if (slides.getCurrentPos() < 120) {
                    // slides.moveSlides(true);
                    while (spintake.getPixelBarPos() < 0.9) {
                        spintake.raiseBar();
                    }
                    spintakeOn = false;
                }*/
            while (!(isStopRequested())) {
                csBot.mecanumDriving(1);
                fourBar.manualControl(true);

                if (gamepad2.x)
                    spintake.stop();
                if (gamepad2.left_bumper && slides.getCurrentPos() > 120)
                    spintake.spin(false);
                else if (gamepad2.left_bumper && slides.getCurrentPos() < 120)
                    slides.moveSlidesToHeightABS(130, 0.7);
                if (gamepad2.right_bumper)
                    spintake.outtake();

                slides.moveSlides(false);

                if (gamepad1.a)
                    slides.resetSlides();
                if (gamepad1.y) {
                    launcher.launch();
                }
                if (gamepad1.b)
                    launcher.resetPos();
                if (gamepad2.dpad_up)
                    fourBar.rotate();
                if (gamepad1.left_trigger > 0.1)
                    hanging.lift();
                if (gamepad1.right_trigger > 0.1)
                    hanging.lower();
                if (gamepad2.back)
                    depositClaw();
                if (gamepad2.left_trigger > 0.1)
                    collectPixel();
                if (gamepad2.a)
                    fourBar.closeClaw();
                if (gamepad2.b)
                    fourBar.openClaw();
                if (gamepad1.dpad_left)
                    spintake.stickOut();
                if (gamepad1.dpad_right)
                    spintake.stickIn();
                if (distance) {
                    if (sensorDistance.lengthDetection() < 6) {
                        drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
                    }
                }
                if (gamepad1.x && distance)
                    distance = false;
                if (gamepad1.x && !(distance))
                    distance = true;
            }
        }
    }

    public void depositClaw() {
        fourBar.lowerFourBar();
        sensorDistance.distanceDetection(hardwareMap,11.8);
        fourBar.openClaw();
        fourBar.raiseFourBar();
    }


    public void collectPixel(){
        spintake.stop();
        fourBar.openClaw();
        spintake.raiseBar();
        slides.moveSlidesToHeightABS(35, 0.4);
        fourBar.closeClaw();
        fourBar.raiseFourBar();
    }
}
