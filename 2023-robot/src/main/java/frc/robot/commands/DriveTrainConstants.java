package frc.robot.commands;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class DriveTrainConstants {

    public static double kRamseteB = 2;
    public static double kRamseteZeta = .7;
    public static double ksVolts = 0.098064;
    public static double kvVoltSecondsPerMeter = 0.00094158;
    public static double kaVoltSecondsSquaredPerMeter = 8.9811E-05;
    public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(.6);
    public static double kPDriveVel = 0.015; // 0.00015161;

}
