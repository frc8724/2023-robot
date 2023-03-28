// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import java.lang.System.Logger.Level;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClawDropCone;
import frc.robot.commands.ClawRollerSet;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.SystemFloorPickUpBack;
import frc.robot.commands.SystemGrabFromHumanPlayerBack;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.commands.SystemPlaceGamePiece;
import frc.robot.commands.SystemZero;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NECMP_Score2GamePieceBack_Level_X_Color_X extends SequentialCommandGroup {
  /** Creates a new Disricts_Score2_Charging_Color_X. */
  public NECMP_Score2GamePieceBack_Level_X_Color_X(int Level, int color) {

    addCommands(new SystemZero());
    addCommands(new SystemPlaceCone(Level));

    addCommands(new ClawDropCone());
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new DriveStraightOnHeading(-0.05, -0.5, 20.0, 0.0 * color),
                new DriveStraightOnHeading(-0.5, 160.0, -10.0 * color),
                new DriveStraightOnHeading(-0.5, -0.05, 20.0, -10.0 * color)),
            new SystemFloorPickUpBack()));

    // Drive Back
    addCommands(new WaitCommand(0.25));
    addCommands(new DriveStraightOnHeading(0.05, 0.5, 20.0, 0.0 * color));
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new DriveStraightOnHeading(0.5, 160.0, 0.0 * color),
                new DriveStraightOnHeading(0.5, 0.05, 20.0, 0.0 * color)),
            new SystemPlaceGamePiece(Level)));
    addCommands(new ClawRollerSet(-.5));

    // addCommands(new DriveStraightOnHeading( 0.05,0.4,30, 0));
    // addCommands(new DriveStraightOnHeading( 0.4,120, 0));
    // addCommands(new DriveStraightOnHeading( 0.4,0.05,30, 0));

  }
}
