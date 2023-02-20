// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveToAprilTagDistance extends CommandBase {
  static final double DRIVE_SLOP_METERS = 0.1;

  double power;
  double distanceInMeters;

  /** Creates a new DriveToAprilTagDistance. */
  public DriveToAprilTagDistance(double power, double distanceInMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
    addRequirements(RobotContainer.limeLight);

    this.power = power;
    this.distanceInMeters = distanceInMeters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive forward or backwards based on too close or too far
    if (RobotContainer.limeLight.getDistanceFromAprilTag() < this.distanceInMeters) {
      RobotContainer.drive.speedRacerDrive(-Math.abs(power), 0.0, false);
      this.distanceInMeters = -Math.abs(this.distanceInMeters);
    } else {
      RobotContainer.drive.speedRacerDrive(Math.abs(power), 0.0, false);
      this.distanceInMeters = Math.abs(this.distanceInMeters);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.speedRacerDrive(0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    // return (Math.abs(this.distanceInMeters -
    // RobotContainer.limeLight.getDistanceFromAprilTag()) < DRIVE_SLOP_METERS);
  }
}
