// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines.Week0;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawPistonSet;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.commands.SystemStowArm;
import frc.robot.commands.SystemZero;
import frc.robot.subsystems.ClawPiston.State;

public class Week0_PlaceCone2_DriveOut extends SequentialCommandGroup {
  /** Creates a new Week0_PlaceCone2_DriveOut. */
  public Week0_PlaceCone2_DriveOut() {
    addCommands(new SystemZero());

    addCommands(new SystemPlaceCone(2));
    addCommands(new ClawPistonSet(State.OPEN));

    addCommands(new ParallelCommandGroup(
        new SystemStowArm(),
        new DriveStraightOnHeading(-0.2, 120.0, 0.0)));
  }
}
