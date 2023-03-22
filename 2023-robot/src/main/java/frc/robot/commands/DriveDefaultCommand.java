// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveDefaultCommand extends CommandBase {
  static final double SLOW_MODE_FACTOR = 0.33;

  Supplier<Double> throttle;
  Supplier<Double> steering;
  Supplier<Boolean> quickTurn;
  Supplier<Boolean> slow;

  /** Creates a new DriveDefaultCommand. */
  public DriveDefaultCommand(Supplier<Double> throttle, Supplier<Double> steering, Supplier<Boolean> quickTurn,
      Supplier<Boolean> slow) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);

    this.throttle = throttle;
    this.steering = steering;
    this.quickTurn = quickTurn;
    this.slow = slow;
  }

  @Override
  public void initialize() {
    // RobotContainer.drive.setBrake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double t = this.throttle.get();
    if (this.slow.get()) {
      t = t * SLOW_MODE_FACTOR;
    }

    RobotContainer.drive.speedRacerDrive(t, this.steering.get(), this.quickTurn.get());
  }
}
