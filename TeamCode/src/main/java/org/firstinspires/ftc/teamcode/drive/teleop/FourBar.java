package org.firstinspires.ftc.teamcode.drive.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBar {
    private Servo fourBar;
    private LinearOpMode opmode;
    public FourBar(HardwareMap hardwareMap, LinearOpMode opmode) {
        fourBar = hardwareMap.get(Servo.class, "launcher");
        this.opmode = opmode;
    }

}
