package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hanging {
    private Servo hangerLeft;
    private Servo hangerRight;
    private LinearOpMode opMode;
    private final double START_POS = 0;
    private final double END_POS = 0.6;

    public Hanging(HardwareMap hardwareMap, LinearOpMode opMode){
        hangerLeft = hardwareMap.get(Servo.class, "hanger2");
        hangerRight = hardwareMap.get(Servo.class, "hanger");
        this.opMode = opMode;
    }
    public void lift(){
        hangerLeft.setPosition(0.69);
        hangerRight.setPosition(0.2);
    }
    public void lower(){
        hangerLeft.setPosition(0.2);
        hangerRight.setPosition(0.69);
    }
}
