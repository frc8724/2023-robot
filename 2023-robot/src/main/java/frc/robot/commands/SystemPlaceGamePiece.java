// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClawPiston;
import frc.robot.subsystems.ClawPiston.State;

public class SystemPlaceGamePiece extends SequentialCommandGroup {
  enum GamePiece {
    Cone,
    Cube
  }

  /** Creates a new SystemPlaceGamePiece. */
  public SystemPlaceGamePiece(int level) {
    addCommands(
        new SelectCommand(
            Map.ofEntries(
                Map.entry(ClawPiston.State.CLOSE, new SystemPlaceCone(level)),
                Map.entry(ClawPiston.State.OPEN, new SystemPlaceCube(level))),
            () -> RobotContainer.clawPiston.get()));
  }
}
