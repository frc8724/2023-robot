package frc.robot.subsystems;

import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBaseSubsystem extends SubsystemBase {

    WPI_TalonFX leftMotor = new WPI_TalonFX(Constants.Talon.DRIVE_LEFT_1);
    WPI_TalonFX rightMotor = new WPI_TalonFX(Constants.Talon.DRIVE_RIGHT_1);
    // The motors on the left side of the drive.
    private final MotorControllerGroup leftMotors = new MotorControllerGroup(
            leftMotor,
            new WPI_TalonFX(Constants.Talon.DRIVE_LEFT_2),
            new WPI_TalonFX(Constants.Talon.DRIVE_LEFT_3));

    // The motors on the right side of the drive.
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(
            rightMotor,
            new WPI_TalonFX(Constants.Talon.DRIVE_RIGHT_2),
            new WPI_TalonFX(Constants.Talon.DRIVE_RIGHT_3));

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

    // The gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    /** Creates a new DriveSubsystem. */
    public DriveBaseSubsystem() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        rightMotors.setInverted(true);

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(
                m_gyro.getRotation2d(), getDistance(leftMotor), getDistance(rightMotor));
    }

    final double meters_per_tick = 1.0;

    double getDistance(TalonFX m) {

        return m.getSelectedSensorPosition() * meters_per_tick;
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                m_gyro.getRotation2d(), getDistance(leftMotor), getDistance(rightMotor));
    }

    /**
     * Returns the currently-estimated pose of the robot.
     * 
     * @return The pose
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     * 
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftMotor.getSelectedSensorVelocity(),
                rightMotor.getSelectedSensorVelocity());
    }

    /**
     * Drives the robot using ardcade controls.
     * 
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     * 
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        m_drive.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        leftMotor.setSelectedSensorPosition(0.0);
        rightMotor.setSelectedSensorPosition(0.0);
    }

    /**
     * Gets the average distance of the two encoders.
     * 
     * @return the average of the two encoder readings
     */
    // public double getAverageEncoderDistance() {
    // return (leftMotor.getDistance() + rightMotor.getDistance()) / 2.0;
    // }

    /**
     * Gets the left drive encoder.
     * 
     * @return the left drive encoder
     */
    // public Encoder getLeftEncoder() {
    // return m_leftEncoder;
    // }

    /**
     * Gets the right drive encoder.
     * 
     * @return the right drive encoder
     */
    // public Encoder getRightEncoder() {
    // return m_rightEncoder;
    // }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     * 
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     * 
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     * 
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }
}
