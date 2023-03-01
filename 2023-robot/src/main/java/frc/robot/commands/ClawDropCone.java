// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClawPiston.State;

public class ClawDropCone extends SequentialCommandGroup {
  /** Creates a new ClawDropCone. */
  public ClawDropCone() {
    addCommands(new ClawPistonSet(State.OPEN));
    addCommands(new WaitCommand(0.5));
  }
}
