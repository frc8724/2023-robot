// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shoulder;

public class ShoulderWaitForPosition extends CommandBase {
  double tolerance;

  /** Creates a new ShoulderWaitForPosition. */
  public ShoulderWaitForPosition() {
    this(Shoulder.POSITION_SLOP);
  }

  public ShoulderWaitForPosition(double t) {
    // addRequirements(RobotContainer.shoulder); // Don't require because we are
    // only reading, not controlling
    tolerance = t;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      RobotContainer.shoulder.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.shoulder.isAtPosition(tolerance);
  }
}
