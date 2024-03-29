// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmBrake.State;

public class ArmWaitForPosition extends CommandBase {
  double tolerance;

  /** Creates a new ArmWaitForPosition. */
  public ArmWaitForPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.arm);
    // addRequirements(RobotContainer.armBrake);
    this(Arm.POSITION_SLOP);
  }

  public ArmWaitForPosition(double tolerance) {
    this.tolerance = tolerance;
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
    // RobotContainer.arm.stop();
    // RobotContainer.armBrake.set(State.CLOSE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.arm.isAtPosition(tolerance);
  }
}
