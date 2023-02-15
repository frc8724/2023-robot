// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * Pass in 2 commands. If the color sensor says it is a cone, run the cone
 * command.
 * Otherwise, run the cube command
 */
public class ClawColorCommand extends CommandBase {
  Command coneCommand;
  Command cubeCommand;
  Command choiceCommand;

  /** Creates a new ClawColorCommand. */
  public ClawColorCommand(Command coneCommand, Command cubeCommand) {
    addRequirements(RobotContainer.clawColor);

    this.coneCommand = coneCommand;
    this.cubeCommand = cubeCommand;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    choiceCommand = RobotContainer.clawColor.isCone() ? coneCommand : cubeCommand;

    choiceCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return choiceCommand.isFinished();
  }
}
