package org.firstinspires.ftc.teamcode.drive.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBar {
    private Servo fourBar;
    private LinearOpMode opmode;
    private final double START_POS = 0.0;
    private final double END_POS = 0.5;
    public FourBar(HardwareMap hardwareMap, LinearOpMode opmode) {
        fourBar = hardwareMap.get(Servo.class, "fourBar");
        this.opmode = opmode;
    }
    public void raise(){
        if (opmode.gamepad2.x){
            fourBar.setPosition(START_POS);
        }
    }
    public void lower(){
        if (opmode.gamepad2.y){
            fourBar.setPosition(END_POS);
        }
    }

}
