// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Week1_PlaceCone3_GetCone_PlaceCone3 extends SequentialCommandGroup {
  /** Creates a new Week1_PlaceCone3_GetCone_PlaceCone3. */
  public Week1_PlaceCone3_GetCone_PlaceCone3() {
    addCommands(new Week1_DriveToCone());
  }
}
