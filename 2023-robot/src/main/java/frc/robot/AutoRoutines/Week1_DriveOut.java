// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.SystemZero;

public class Week1_DriveOut extends SequentialCommandGroup {
  /** Creates a new Week0_DriveOut. */
  public Week1_DriveOut() {
    addCommands(new SystemZero());
    addCommands(new DriveStraightOnHeading(-0.2, 160.0, 0.0));
  }
}
