// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Week3_PlaceConeGetAnother_Charging_Level_3_Color_Blue extends SequentialCommandGroup {
  /** Creates a new Week3_PlaceConeGetAnother_Charging_Level_3_Color_Blue. */
  public Week3_PlaceConeGetAnother_Charging_Level_3_Color_Blue() {
    addCommands(new Week3_PlaceCone_Charging_Level_X_Color_X(3, -1));
  }
}
