// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawPistonSet;
import frc.robot.commands.ClawRollerSet;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.ShoulderGoto;
import frc.robot.commands.ShoulderWaitForPosition;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.subsystems.ClawPiston.State;
import frc.robot.subsystems.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Drive_Place2ndCubeBump extends SequentialCommandGroup {
  /** Creates a new Drive_Place2ndCubeBump. */
  public Drive_Place2ndCubeBump(int Level, int Color) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveStraightOnHeading(0.2, 0.4, 10, 0 * Color));
    addCommands(new DriveStraightOnHeading(0.4, 115, 0 * Color));

    addCommands(new DriveStraightOnHeading(0.4, 0.15, 10, 0 * Color));
    addCommands(new DriveStraightOnHeading(0.15, 15, 0 * Color));// go over bump
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new SystemPlaceCone(Level),
                new ShoulderGoto(Shoulder.LEVEL_X_SCORE[Level]),
                new ShoulderWaitForPosition(),

                new ClawRollerSet(-.5),
                new ClawPistonSet(State.CLOSE)),
            new SequentialCommandGroup(
                new DriveStraightOnHeading(0.15, 0.4, 15, 0 * Color),
                new DriveStraightOnHeading(0.4, 50, -15 * Color),
                new DriveStraightOnHeading(0.4, 0.05, 15, 0 * Color))));

  }
}
