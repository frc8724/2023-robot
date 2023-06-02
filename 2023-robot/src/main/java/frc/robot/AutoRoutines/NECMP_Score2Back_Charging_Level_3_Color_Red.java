// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveStraightOnHeading;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NECMP_Score2Back_Charging_Level_3_Color_Red extends SequentialCommandGroup {
  /** Creates a new NECMP_Score2Back_Charging_Level_3_Color_Red. */
  public NECMP_Score2Back_Charging_Level_3_Color_Red() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new NECMP_Score2_Charge_Level_X_Color_X(3,1));

    // addCommands(new NECMP_Score2Back_Charging_Level_X_Color_X(3, 1));
    // addCommands(new NECMP_DriveToThirdGamePiece(1));
    
  }
}
