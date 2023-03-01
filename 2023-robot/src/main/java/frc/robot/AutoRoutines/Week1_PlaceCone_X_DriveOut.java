// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawDropCone;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.commands.SystemStowArm;
import frc.robot.commands.SystemZero;

public class Week1_PlaceCone_X_DriveOut extends SequentialCommandGroup {
  /** Creates a new Week1_PlaceCone2_DriveOut. */
  public Week1_PlaceCone_X_DriveOut(int level) {
    addCommands(new SystemZero());

    addCommands(new SystemPlaceCone(level));

    addCommands(new ClawDropCone());

    addCommands(
        new ParallelCommandGroup(
            new Week1_DriveOut(),
            new SystemStowArm()));
  }
}
