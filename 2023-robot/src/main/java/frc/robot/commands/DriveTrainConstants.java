package frc.robot.commands;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class DriveTrainConstants {

    public static double kRamseteB = 2;
    public static double kRamseteZeta = .7;

    public static double ksVolts = 0.071169; // 0.098064;
    public static double kvVoltSecondsPerMeter = 1.9857; // 0.00094158;
    public static double kaVoltSecondsSquaredPerMeter = 0.072955; // 8.9811E-05;
    public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(.6);
    public static double kPDriveVel = 0.0083892; // 0.015; // 0.00015161;
    public static double kDDriveVel = 0.1;
}
