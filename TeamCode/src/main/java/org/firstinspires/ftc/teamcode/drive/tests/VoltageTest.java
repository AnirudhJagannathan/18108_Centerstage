package org.firstinspires.ftc.teamcode.drive.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Sensors.SensorDistance;
import org.firstinspires.ftc.teamcode.drive.teleop.CenterstageBot;
import org.firstinspires.ftc.teamcode.drive.teleop.FourBar;
import org.firstinspires.ftc.teamcode.drive.teleop.Hanging;
import org.firstinspires.ftc.teamcode.drive.teleop.Launcher;
import org.firstinspires.ftc.teamcode.drive.teleop.Slides;
import org.firstinspires.ftc.teamcode.drive.teleop.Spintake;
import org.firstinspires.ftc.teamcode.drive.teleop.TankDrive;

@TeleOp
public class VoltageTest extends LinearOpMode {
    private SampleMecanumDrive drive;
    private CenterstageBot csBot;
    private TankDrive tankDrive;
    private Launcher launcher;
    private Spintake spintake;
    private Slides slides;
    private FourBar fourBar;
    private Hanging hanging;
    private SensorDistance sensorDistance;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

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

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

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
            csBot.mecanumDriving(1);

            if (gamepad2.left_bumper)
                spintake.spin(false);

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
            if (gamepad1.right_trigger > 0.1) {
                hanging.lower();
                fourBar.cutPower();
            }
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


            if (gamepad2.left_trigger > 0.1)
                spintake.stickIn();
            if (gamepad2.right_trigger > 0.1)

                spintake.stickOut();

            if (gamepad2.start) {
                spintake.stickIntake();
                sleep(250);
            }

            double currentVoltage = hardwareMap.voltageSensor.get("Control Hub").getVoltage();
            telemetry.addData("VoltageMitigation:", currentVoltage);
            // telemetry.addData("VoltageRaw:", currentVoltage + Math.pow(-1, Math.round(Math.random())) * 0.45 * Math.random() - Math.random());
            telemetry.update();
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
