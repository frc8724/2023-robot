// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SystemZero;

public class Week3_PlaceCone_Charging_Level_X_Color_X extends SequentialCommandGroup {
  public Week3_PlaceCone_Charging_Level_X_Color_X(int Level, int color) {
    addCommands(new SystemZero());

    addCommands(new Week3_PlaceCone_GrabAnother_Level_X_Color_X(Level, color));

  }
}
