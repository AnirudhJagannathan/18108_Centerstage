package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

public class Hanging {
    private Servo hangerLeft;
    private Servo hangerRight;
    private LinearOpMode opMode;
    private final double START_POS = 0.04;
    private final double MID_POS = 0.5;
    private final double END_POS = 0.98;

    public Hanging(HardwareMap hardwareMap, LinearOpMode opMode){
        hangerLeft = hardwareMap.get(Servo.class, "hanger2");
        hangerRight = hardwareMap.get(Servo.class, "hanger");
        this.opMode = opMode;
    }
    public void lift(){
        hangerLeft.setPosition(START_POS);
        hangerRight.setPosition(END_POS);

        ((PwmControl) hangerLeft).setPwmDisable();
        ((PwmControl) hangerRight).setPwmDisable();
    }
    public void lower(){
        hangerLeft.setPosition(END_POS);
        hangerRight.setPosition(START_POS);
    }

    public void smallerLift() {
        hangerLeft.setPosition(MID_POS);
        hangerRight.setPosition(MID_POS);
    }

    public Servo getHangerLeft() {
        return hangerLeft;
    }

    public Servo getHangerRight() {
        return hangerRight;
    }
}
