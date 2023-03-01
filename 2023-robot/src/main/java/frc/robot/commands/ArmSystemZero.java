// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmBrake.State;

public class ArmSystemZero extends SequentialCommandGroup {
  /** Creates a new ArmSystemZero. */
  public ArmSystemZero() {
    addCommands(new ArmBrakeSet(State.OPEN));
    addCommands(new WaitCommand(0.1));
    addCommands(new ArmZero().withTimeout(1.0));
    addCommands(new ArmBrakeSet(State.CLOSE));
  }
}
