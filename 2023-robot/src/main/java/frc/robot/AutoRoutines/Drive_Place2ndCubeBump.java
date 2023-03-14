// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.SystemPlaceCone;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Drive_Place2ndCubeBump extends SequentialCommandGroup {
  /** Creates a new Drive_Place2ndCubeBump. */
  public Drive_Place2ndCubeBump(int Level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveStraightOnHeading(0.2, 0.4, 10, 0));
    addCommands(new DriveStraightOnHeading(0.4, 110, 0));

    addCommands(new DriveStraightOnHeading(0.4, 0.15, 10, 0));
    addCommands(new DriveStraightOnHeading(0.15, 15, 0));// go over bump
    addCommands(
        new ParallelCommandGroup(
            new SystemPlaceCone(Level),
            new SequentialCommandGroup(
                new DriveStraightOnHeading(0.15, 0.4, 15, 0),
                new DriveStraightOnHeading(0.4, 45, -2),
                new DriveStraightOnHeading(0.4, 0.2, 15, 0))));
  }
}
