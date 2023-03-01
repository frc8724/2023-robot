// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveCenterTarget extends CommandBase {
  Supplier<Double> throttle;

  /** Creates a new DriveCenterTarget. */
  public DriveCenterTarget(Supplier<Double> throttle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
    addRequirements(RobotContainer.targeting);
    this.throttle = throttle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.targeting.hasTarget()) {
      double currentHeading = RobotContainer.drive.getHeading();
      double desiredHeading = RobotContainer.targeting.getHorizontalOffset();
      RobotContainer.drive.setDesiredHeading(currentHeading + desiredHeading);

      double t = throttle.get();
      RobotContainer.drive.speedRacerDrive(t, 0.0, true);
    } else {
      RobotContainer.drive.speedRacerDrive(0.0, 0.0, false);
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
