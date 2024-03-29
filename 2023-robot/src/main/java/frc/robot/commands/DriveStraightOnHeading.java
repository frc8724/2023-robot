package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBaseSubsystem;

public class DriveStraightOnHeading extends CommandBase {

    // double m_targetPower;
    double m_startingPower;
    double m_finalPower;

    double m_desiredDisplacement;
    double m_desiredHeading;

    public enum DistanceUnits {
        ENCODER_TICKS, INCHES
    };

    public DriveStraightOnHeading(double arg_targetSpeed, double arg_distance) {
        this(arg_targetSpeed, DistanceUnits.INCHES, arg_distance, RobotContainer.drive.getHeading());
    }

    public DriveStraightOnHeading(double arg_targetSpeed, double arg_distance, double heading) {
        this(arg_targetSpeed, DistanceUnits.INCHES, arg_distance, heading);
    }

    public DriveStraightOnHeading(double start, double finish, double arg_distance, double heading) {
        this(start, finish, DistanceUnits.INCHES, arg_distance, heading);
    }

    /**
     * 
     * @param arg_targetPower +/- motor power [-1.0, +1.0]
     * @param arg_distance    Distance in encoder counts
     */
    public DriveStraightOnHeading(double arg_targetSpeed, DistanceUnits units, double arg_distance, double heading) {
        this(arg_targetSpeed, arg_targetSpeed, units, arg_distance, heading);
    }

    public DriveStraightOnHeading(
            double arg_startingPower,
            double arg_finalPower,
            DistanceUnits units,
            double arg_distance,
            double heading) {
        addRequirements(RobotContainer.drive);

        if (units == DistanceUnits.INCHES) {
            arg_distance = arg_distance / DriveBaseSubsystem.DISTANCE_PER_PULSE_IN_INCHES;
        }
        m_startingPower = arg_startingPower;
        m_finalPower = arg_finalPower;

        m_desiredDisplacement = Math.abs(arg_distance);
        m_desiredHeading = heading;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        RobotContainer.drive.saveInitialWheelDistance();
        RobotContainer.drive.setDesiredHeading(m_desiredHeading);
        // System.out.println("Starting Routine: Drive Straight On Heading");
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double displacement = Math.abs(RobotContainer.drive.getWheelDistance());
        double power = m_startingPower + displacement / m_desiredDisplacement * (this.m_finalPower - m_startingPower);
        RobotContainer.drive.speedRacerDrive(power);
        // RobotContainer.drive.speedRacerDrive(power, 0, false);

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        int displacement = (int) RobotContainer.drive.getWheelDistance();

        displacement = Math.abs(displacement);
        // System.out.println("displacement" + displacement);

        return (displacement >= m_desiredDisplacement);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drive.stop();
    }

}
