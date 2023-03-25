package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBaseSubsystem;

public class DriveArcingTurn extends CommandBase {

  // double m_targetPower;
  double m_startingPower;
  double m_finalPower;

  double m_desiredDisplacement;
  double m_startingHeading;
  double m_finalHeading;

  public DriveArcingTurn(
      double arg_startingPower,
      double arg_finalPower,
      double arg_distance,
      double headingStart,
      double headingFinal) {
    addRequirements(RobotContainer.drive);

    arg_distance = arg_distance / DriveBaseSubsystem.DISTANCE_PER_PULSE_IN_INCHES;
    m_startingPower = arg_startingPower;
    m_finalPower = arg_finalPower;

    m_desiredDisplacement = Math.abs(arg_distance);
    m_startingHeading = headingStart;
    m_finalHeading = headingFinal;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotContainer.drive.saveInitialWheelDistance();
    RobotContainer.drive.setDesiredHeading(m_startingHeading);
    // System.out.println("Starting Routine: Drive Straight On Heading");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double displacement = Math.abs(RobotContainer.drive.getWheelDistance());
    double power = m_startingPower + displacement / m_desiredDisplacement * (this.m_finalPower - m_startingPower);
    double heading = m_startingHeading
        + displacement / m_desiredDisplacement * (this.m_finalHeading - m_startingHeading);

    RobotContainer.drive.setDesiredHeading(heading);
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
