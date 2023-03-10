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
    // drive to center line
    addCommands(new DriveStraightOnHeading(-0.05, -0.5, 20.0, 0.0));
    addCommands(new DriveStraightOnHeading(-0.5, 160.0, 2.0));
    addCommands(new DriveStraightOnHeading(-0.5, -0.2, 20.0, 2.0));

    // turn and get cone/cube
    addCommands(new DriveStraightOnHeading(-0.2, 25.0, 50.0));
    
    addCommands(new DriveStraightOnHeading(0.05,0.2, 5.0, 85.0));
    addCommands(new DriveStraightOnHeading(0.2, 15.0, 85.0));

    // drive back to grid
    // addCommands(new DriveStraightOnHeading(0.05, 20.0, -5.0));
    addCommands(new DriveStraightOnHeading(0.2, 0.5, 20.0,-5.0));
    addCommands(new DriveStraightOnHeading(0.5, 120.0, 0.0));

    addCommands(new DriveStraightOnHeading(0.5, 78.0, 6.0));

    addCommands(new DriveStraightOnHeading(0.5, 0.05, 20.0, 6.0));

  }
}
