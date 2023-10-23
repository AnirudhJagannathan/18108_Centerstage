package org.firstinspires.ftc.teamcode.drive.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBar {
    private Servo fourBar;
    private final double START_POS = 0.95;
    private final double END_POS = 0;
    public FourBar(HardwareMap hardwareMap) {
        fourBar = hardwareMap.get(Servo.class, "fourBar");
    }
    public void resetPos() {
        fourBar.setPosition(START_POS);
    }
    public void rotate() {
        fourBar.setPosition(END_POS);
    }
}
