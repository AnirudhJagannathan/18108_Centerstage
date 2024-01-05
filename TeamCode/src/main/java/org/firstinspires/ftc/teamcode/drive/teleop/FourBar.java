package org.firstinspires.ftc.teamcode.drive.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class FourBar {
    private Servo fourBar;
    private Servo fourBar2;
    private Servo claw;

    static final double INCREMENT = 0.02;

    // private CRServo fourBarRight;
    private LinearOpMode opmode;
    private final double START_POS = 1.0;
    private final double END_POS = 0.0;
    private final double POS_START = 0;
    private final double POS_END = 0;
    // private final double WRIST_END = 0.6;
    // private final double WRIST_START = 1.0;

    double position = 0.88;
    double position2 = 0;
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
        fourBar.setPosition(0.73);
        fourBar2.setPosition(0.12);
        opmode.telemetry.addData("Position", fourBar.getPosition());
        // fourBarRight.setPower(1.0);
    }
    public void rotatePos(double pos) {
        while (pos < fourBar.getPosition()) {
            position -= INCREMENT;
            fourBar.setPosition(position);
            fourBar2.setPosition(position);
        }
        while (pos > fourBar.getPosition()) {
            position += INCREMENT;
            fourBar.setPosition(position);
            fourBar2.setPosition(position);
        }
        if (pos == fourBar.getPosition()) {
            fourBar2.setPosition(pos);
            fourBar.setPosition(pos);
        }
    }
    public void closeClaw() {
        claw.setPosition(0.7);
        // fourBarRight.setPower(1.0);
    }
    public void openClaw() {
        claw.setPosition(0.59);
    }

    public void manualControl(boolean spintakeOn) {
        if (opmode.gamepad2.left_stick_y > 0) {
            // Keep stepping up until we hit the max value.
            position -= INCREMENT;
            position2 += INCREMENT;
            if (position <= 0.5 && position2 >= 0.5) {
                position = 0;// Switch ramp direction
                position2 = 0.88;
            }

        }
        else if(opmode.gamepad2.left_stick_y < 0){
            position += INCREMENT;
            position2 -= INCREMENT;
            if (position >= 0.88 && position2 <= 0) {
                position = 0.88;// Switch ramp direction
                position2 = 0;
            }
            fourBar.setPosition(position);
            fourBar2.setPosition(position2);
        }
        opmode.telemetry.addData("position:", position);
        opmode.telemetry.addData("position2:", position2);
        /* if (spintakeOn) {

        } else {
            fourBar.getController().pwmDisable();
        }
         */
    }

    public void raiseFourBar() {
        fourBar.setPosition(0.85);
        fourBar2.setPosition(0);
        position = 0.85;
        position2 = 0;
    }
    public void lowerFourBar() {
        fourBar.setPosition(0);
        fourBar2.setPosition(0.85);
        position = 0;
        position2 = 0.85;
    }
    public void pixelFourBar() {
        fourBar.setPosition(0.07);
        fourBar2.setPosition(0.78);
        position = 0.07;
        position2 = 0.78;
    }

    public void cutPower() {
        ((PwmControl) claw).setPwmDisable();
        ((PwmControl) fourBar).setPwmDisable();
        ((PwmControl) fourBar2).setPwmDisable();
    }

    public Servo getClaw() {
        return claw;
    }

    public Servo getFourBar() {
        return fourBar;
    }

    public Servo getFourBar2() {
        return fourBar2;
    }
}
