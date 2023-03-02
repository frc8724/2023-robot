// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraightOnHeading;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Week3_DriveToCone extends SequentialCommandGroup {
  /** Creates a new Week1_DriveToCone. */
  public Week3_DriveToCone() {
    addCommands(new DriveStraightOnHeading(-0.2, 120.0, 0.0));
    addCommands(new DriveStraightOnHeading(-0.2, 50.0, 60.0));
    addCommands(new DriveStraightOnHeading(0.2, 80.0, 70.0));

  }
}
