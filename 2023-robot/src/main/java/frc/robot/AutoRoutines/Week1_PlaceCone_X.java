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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Week1_PlaceCone_X extends SequentialCommandGroup {
  /** Creates a new Week1_PlaceCone_X. */
  public Week1_PlaceCone_X(int level) {
    addCommands(new SystemZero());

    addCommands(new SystemPlaceCone(level));

    addCommands(new ClawDropCone());

    addCommands(new SystemStowArm());

  }
}
