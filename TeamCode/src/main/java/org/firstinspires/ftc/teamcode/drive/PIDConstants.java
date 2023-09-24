package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class PIDConstants {
    public static double Kp = 1.8;  // Proportional gain
    public static double Ki = 0.0;   // Integral gain ( KEEP THIS 0 )
    public static double Kd = 0.03;   // Derivative gain
}