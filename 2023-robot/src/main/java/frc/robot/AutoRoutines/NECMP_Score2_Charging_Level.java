// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveBrakeMode;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.SystemPlaceGamePiece;
import frc.robot.commands.SystemStowArm;
import frc.robot.commands.SystemZero;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NECMP_Score2_Charging_Level extends SequentialCommandGroup {
  /** Creates a new NECMP_Score2_Charging_Level. */
  public NECMP_Score2_Charging_Level() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(new SystemZero());
    // addCommands(new SystemPlaceGamePiece(2));
    addCommands(
        new ParallelCommandGroup(
            new SystemStowArm(),
            new SequentialCommandGroup(
                new WaitCommand(1),
                new DriveStraightOnHeading(-0.05, -0.25, 30, -70),
                new DriveStraightOnHeading(-0.25, -0.4, 10, -70),
                new DriveBrakeMode(true),
                new DriveStraightOnHeading(-0.4, -0.25, 45, 0),
                new DriveStraightOnHeading(-0.25, -0.05, 38, 0))));

  }
}
