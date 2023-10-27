package org.firstinspires.ftc.teamcode.drive.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBar {
    private Servo fourbarLeft;
    private Servo fourBarRight;
    private LinearOpMode opmode;
    private final double START_POS = 0.95;
    private final double END_POS = 0;
    public FourBar(HardwareMap hardwareMap, LinearOpMode opmode) {
        fourbarLeft = hardwareMap.get(Servo.class, "fourBarLeft");
        fourBarRight = hardwareMap.get(Servo.class, "fourBarRight");
        this.opmode = opmode;
    }
    public void resetPos(){
        fourbarLeft.setPosition(START_POS);
        fourBarRight.setPosition(START_POS);
    }
    public void rotate(){
        fourbarLeft.setPosition(END_POS);
        fourBarRight.setPosition(END_POS);
    }

}
