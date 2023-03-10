// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.mayheminc.util.LEDLights.PatternID;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LedLightsSet extends CommandBase {
  PatternID pattern;

  public LedLightsSet(PatternID p) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.ledLights);
    pattern = p;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.ledLights.set(PatternID.RED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.ledLights.set(pattern);
  }
}
