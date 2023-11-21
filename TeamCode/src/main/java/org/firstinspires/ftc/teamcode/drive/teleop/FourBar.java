package org.firstinspires.ftc.teamcode.drive.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBar {
    private Servo fourBar;
    private Servo wrist;
    private Servo claw;

    static final double INCREMENT   = 0.01;

    // private CRServo fourBarRight;
    private LinearOpMode opmode;
    private final double START_POS = 0.3;
    private final double END_POS = 0;
    // private final double WRIST_END = 0.6;
    // private final double WRIST_START = 1.0;

    double position = 1.0;
    public FourBar(HardwareMap hardwareMap, LinearOpMode opmode) {
        fourBar = hardwareMap.get(Servo.class, "fourBar");
        claw = hardwareMap.get(Servo.class, "claw");
        // fourBarRight = hardwareMap.get(CRServo.class, "fourBarRight");
        this.opmode = opmode;
    }
    public void resetPos(){
        fourBar.setPosition(END_POS);
        // fourBarRight.setPower(-0.25);
    }
    public void rotate() {
        fourBar.setPosition(START_POS);
        // fourBarRight.setPower(1.0);
    }
    public void closeClaw() {
        claw.setPosition(0.7);
        // fourBarRight.setPower(1.0);
    }
    public void openClaw() {
        claw.setPosition(0.5);
    }

    public void manualControl() {
        if (opmode.gamepad2.left_stick_y > 0.5) {
            // Keep stepping up until we hit the max value.
            position -= INCREMENT ;
            if (position <= 0.5) {
                position = 0;// Switch ramp direction
            }
        }
        else if(opmode.gamepad2.left_stick_y < 0){
            position += INCREMENT ;
            if (position >= 1.0) {
                position = 1.0;// Switch ramp direction
            }
        }
        fourBar.setPosition(position);
    }

}
