// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmBrake;
import frc.robot.subsystems.ArmBrake.State;

public class ArmSystemZero extends SequentialCommandGroup {
  /** Creates a new ArmSystemZero. */
  public ArmSystemZero() {
    addCommands(new ArmBrakeSet(State.OPEN));
    addCommands(new WaitCommand(ArmBrake.OPEN_TIME_SEC));

    addCommands(new ParallelRaceGroup(new ArmZero(), new WaitCommand(1.0)));
    addCommands(new ArmBrakeSet(State.CLOSE));
  }
}
