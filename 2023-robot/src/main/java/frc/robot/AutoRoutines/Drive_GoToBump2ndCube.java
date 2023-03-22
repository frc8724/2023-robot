// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraightOnHeading;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Drive_GoToBump2ndCube extends SequentialCommandGroup {
  /** Creates a new Drive_GoToBump2ndCube. */
  public Drive_GoToBump2ndCube(int Color) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveStraightOnHeading(-0.05, -0.4, 15, 0 * Color));
    addCommands(new DriveStraightOnHeading(-0.4, 33, 0 * Color));// get to bump
    addCommands(new DriveStraightOnHeading(-0.4, -0.15, 10, 0 * Color));
    addCommands(new DriveStraightOnHeading(-0.15, 15, 0 * Color));// go over bump
    addCommands(new DriveStraightOnHeading(-0.15, -0.4, 15, 0 * Color));
    addCommands(new DriveStraightOnHeading(-0.4, 137, 24 * Color));// long distance
    addCommands(new DriveStraightOnHeading(-0.4, -0.1, 20, 82 * Color));// aim to cube
    addCommands(new DriveStraightOnHeading(0.05, 0.2, 15, 80 * Color));// get cube

  }
}
