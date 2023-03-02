// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Week1_PlaceCone_3 extends SequentialCommandGroup {
  /** Creates a new Week1_PlaceCone_3. */
  public Week1_PlaceCone_3() {
    addCommands(new SystemPlaceCone_X(3));
  }
}
