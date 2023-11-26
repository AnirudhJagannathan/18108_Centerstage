package org.firstinspires.ftc.teamcode.drive.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class FourBar {
    private Servo fourBar;
    private Servo fourBar2;
    private Servo claw;

    static final double INCREMENT = 0.01;

    // private CRServo fourBarRight;
    private LinearOpMode opmode;
    private final double START_POS = 1.0;
    private final double END_POS = 0.0;
    private final double POS_START = 0;
    private final double POS_END = 0;
    // private final double WRIST_END = 0.6;
    // private final double WRIST_START = 1.0;

    double position = 1.0;
    public FourBar(HardwareMap hardwareMap, LinearOpMode opmode) {
        fourBar = hardwareMap.get(Servo.class, "fourBar");
        fourBar2 = hardwareMap.get(Servo.class, "fourBar2");
        claw = hardwareMap.get(Servo.class, "claw");
        // fourBarRight = hardwareMap.get(CRServo.class, "fourBarRight");
        this.opmode = opmode;
    }
    public void resetPos(){
        fourBar.setPosition(START_POS);
        fourBar2.setPosition(START_POS);
        opmode.telemetry.addData("Position", fourBar.getPosition());
    }
    public void rotate() {
        fourBar.setPosition(END_POS);
        fourBar2.setPosition(END_POS);
        opmode.telemetry.addData("Position", fourBar.getPosition());
        // fourBarRight.setPower(1.0);
    }
    public void rotatePos(double pos) {
        fourBar.setPosition(pos);
        fourBar2.setPosition(pos);
    }
    public void closeClaw() {
        claw.setPosition(0.7);
        // fourBarRight.setPower(1.0);
    }
    public void openClaw() {
        claw.setPosition(0.61);
    }

    public void manualControl() {
        if (opmode.gamepad2.left_stick_y > 0) {
            // Keep stepping up until we hit the max value.
            position -= INCREMENT ;
            if (position <= 0.5) {
                position = 0;// Switch ramp direction
            }
        }
        else if(opmode.gamepad2.left_stick_y < 0){
            position += INCREMENT;
            if (position >= 1.0) {
                position = 1.0;// Switch ramp direction
            }
        }
        opmode.telemetry.addData("position:", position);
        fourBar.setPosition(position);
        fourBar2.setPosition(position);
    }
}
