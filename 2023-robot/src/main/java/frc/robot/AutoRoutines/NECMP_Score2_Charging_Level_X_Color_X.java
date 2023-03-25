// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawDropCone;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.SystemFloorPickUpBack;
import frc.robot.commands.SystemGrabFromHumanPlayerBack;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.commands.SystemZero;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NECMP_Score2_Charging_Level_X_Color_X extends SequentialCommandGroup {
  /** Creates a new Disricts_Score2_Charging_Color_X. */
  public NECMP_Score2_Charging_Level_X_Color_X(int Level, int color) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SystemZero());
    // addCommands(new SystemPlaceCone(Level));

    // addCommands(new ClawDropCone());
    addCommands(new DriveStraightOnHeading(-0.05, -0.4, 20.0, 0.0 * color));
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new DriveStraightOnHeading(-0.4, 160.0, -8.0 * color),
                new DriveStraightOnHeading(-0.4, -0.05, 20.0, -8.0 * color)),
            new SystemFloorPickUpBack()));

    // addCommands(new DriveStraightOnHeading( 0.05,0.4,30, 0));
    // addCommands(new DriveStraightOnHeading( 0.4,120, 0));
    // addCommands(new DriveStraightOnHeading( 0.4,0.05,30, 0));

  }
}