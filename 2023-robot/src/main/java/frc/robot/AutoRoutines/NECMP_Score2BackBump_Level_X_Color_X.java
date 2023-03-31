// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmGoto;
import frc.robot.commands.ClawDropCone;
import frc.robot.commands.ClawRollerSet;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.SystemFloorPickUpBack;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.commands.SystemPlaceCubeBack;
import frc.robot.commands.SystemZero;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NECMP_Score2BackBump_Level_X_Color_X extends SequentialCommandGroup {
  /** Creates a new NECMP_Score2BackBump_Level_X_Color_X. */
  public NECMP_Score2BackBump_Level_X_Color_X(int level, int color) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SystemZero());
    addCommands(new SystemPlaceCone(level));
    addCommands(new ClawDropCone());

    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new DriveStraightOnHeading(-0.05, -0.4, 15, 0 * color),
                new DriveStraightOnHeading(-0.4, 33, 0 * color),
                new DriveStraightOnHeading(-0.4, -0.15, 10, 0 * color),
                new DriveStraightOnHeading(-0.15, 15, 5 * color)),
            new ArmGoto(Arm.ALMOST_STOW)));
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new DriveStraightOnHeading(-0.15, -0.35, 15, 5 * color),
                new DriveStraightOnHeading(-0.35, 95, 5 * color)),
            new SystemFloorPickUpBack()));
    addCommands(
        new DriveStraightOnHeading(-0.35, -0.1, 15, 0 * color),
        new ClawRollerSet(ClawRollers.SUCK_IN),
        new DriveStraightOnHeading(-0.1, 5, 0 * color));



    addCommands(new DriveStraightOnHeading(0.05, 0.4, 20, 0 * color));
    addCommands(new DriveStraightOnHeading(0.4, 95, 0 * color));
    addCommands(new DriveStraightOnHeading(0.4, 0.15, 15, 0 * color));
    addCommands(new DriveStraightOnHeading(0.15, 15, 0 * color));

    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new DriveStraightOnHeading(0.15, 0.4, 10, -10 * color),
                new DriveStraightOnHeading(0.4, 23, -10 * color),
                new DriveStraightOnHeading(0.4, 0.05, 25, 0 * color)),
            new SystemPlaceCubeBack(level)));
    addCommands(new ClawRollerSet(ClawRollers.SPIT));

  }
}
