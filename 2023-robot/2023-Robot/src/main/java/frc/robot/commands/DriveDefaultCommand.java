// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveDefaultCommand extends CommandBase {
  Supplier<Double> throttle;
  Supplier<Double> steering;
  Supplier<Boolean> slow;

  /** Creates a new DriveDefaultCommand. */
  public DriveDefaultCommand(Supplier<Double> throttle, Supplier<Double> steering, Supplier<Boolean> slow) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
    this.throttle = throttle;
    this.steering = steering;
    this.slow = slow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drive.speedRacerDrive(this.throttle.get(), this.steering.get(), this.slow.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
