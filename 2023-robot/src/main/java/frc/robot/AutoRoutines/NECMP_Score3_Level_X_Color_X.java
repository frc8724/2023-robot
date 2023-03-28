// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NECMP_Score3_Level_X_Color_X extends SequentialCommandGroup {
  /** Creates a new NECMP_Score3_Level_X_Color_X. */
  public NECMP_Score3_Level_X_Color_X(int Level,int color) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new NECMP_Score2GamePieceBack_Level_X_Color_X(Level,color));
    addCommands(new NECMP_Drive3GamePiece(color));
  }
}
