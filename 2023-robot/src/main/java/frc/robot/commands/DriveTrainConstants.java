package frc.robot.commands;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class DriveTrainConstants {

    public static double kRamseteB = 2;
    public static double kRamseteZeta = .7;
    public static double ksVolts = .2;
    public static double kvVoltSecondsPerMeter = 2.0;
    public static double kaVoltSecondsSquaredPerMeter = .2;
    public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(.7);
    public static double kPDriveVel = 8.5;

}
