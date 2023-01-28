// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveCenterTarget extends CommandBase {

  /** Creates a new DriveCenterTarget. */
  public DriveCenterTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
    addRequirements(RobotContainer.targeting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.targeting.hasTarget()) {
      RobotContainer.drive.speedRacerDrive(0.0, getRotationSpeed(), true);
    } else {
      RobotContainer.drive.speedRacerDrive(0.0, 0.0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.speedRacerDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  double getRotationSpeed() {
    double h = RobotContainer.targeting.getCorrection();
    return h;
  }
}
