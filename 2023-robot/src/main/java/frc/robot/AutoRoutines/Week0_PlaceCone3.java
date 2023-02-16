// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SystemPlaceCone;

public class Week0_PlaceCone3 extends SequentialCommandGroup {
  /** Creates a new Week0_PlaceCone3. */
  public Week0_PlaceCone3() {
    addCommands(new SystemPlaceCone(3));
  }
}
