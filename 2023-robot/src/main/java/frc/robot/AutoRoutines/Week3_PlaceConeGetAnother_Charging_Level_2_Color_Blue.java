// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Week3_PlaceConeGetAnother_Charging_Level_2_Color_Blue extends SequentialCommandGroup {
  /** Creates a new Week3_PlaceConeGetAnother_Charging_Level_2_Color_Blue. */
  public Week3_PlaceConeGetAnother_Charging_Level_2_Color_Blue() {
    addCommands(new Week3_PlaceCone_Charging_Level_X_Color_X(2, -1));
  }
}
