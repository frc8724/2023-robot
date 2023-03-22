// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawDropCone;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.ShoulderGoto;
import frc.robot.commands.ShoulderWaitForPosition;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.commands.SystemStowArm;
import frc.robot.commands.SystemZero;
import frc.robot.subsystems.Shoulder;

public class SystemPlaceCone_X_DriveOut extends SequentialCommandGroup {
  /** Creates a new Week1_PlaceCone2_DriveOut. */
  public SystemPlaceCone_X_DriveOut(int level) {
    addCommands(new SystemZero());

    addCommands(new SystemPlaceCone(level));

    addCommands(new ClawDropCone());

    addCommands(new ShoulderGoto(Shoulder.LEVEL_X_PRESCORE[level]));
    addCommands(new ShoulderWaitForPosition());

    addCommands(new ParallelCommandGroup(
        new SystemStowArm(),
        new SequentialCommandGroup(
            new DriveStraightOnHeading(-0.05, -.2, 20.0, 0.0),
            new DriveStraightOnHeading(-0.2, 80.0, 0.0),
            new DriveStraightOnHeading(-0.2, -0.05, 20.0, 0.0))));
  }
}
