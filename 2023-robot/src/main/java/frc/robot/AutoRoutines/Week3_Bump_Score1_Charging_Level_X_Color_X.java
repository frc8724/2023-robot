// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmSystemGoTo;
import frc.robot.commands.ArmZero;
import frc.robot.commands.ClawDropCone;
import frc.robot.commands.ClawPistonSet;
import frc.robot.commands.ClawRollerSet;
import frc.robot.commands.DriveBrakeMode;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.ShoulderGoto;
import frc.robot.commands.ShoulderWaitForPosition;
import frc.robot.commands.SystemFloorPickUp;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.commands.SystemZero;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawPiston;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.ClawPiston.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Week3_Bump_Score1_Charging_Level_X_Color_X extends SequentialCommandGroup {
  /** Creates a new Week3_Bump_Score1_Charging_Level_X_Color_X. */
  public Week3_Bump_Score1_Charging_Level_X_Color_X(int Level, int color) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SystemZero());
    addCommands(new SystemPlaceCone(Level));

    addCommands(new ClawDropCone());

    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ArmSystemGoTo(Arm.ALMOST_STOW),
                new SystemFloorPickUp()),
            new Drive_GoToBump2ndCube(color)));
    addCommands(new ClawPistonSet(State.CLOSE));
    addCommands(new ParallelCommandGroup(
        // stutter the claw rollers to settle the cone
        new SequentialCommandGroup(
            new WaitCommand(1.0),
            new ClawRollerSet(0.2),
            new WaitCommand(1.0),
            new ClawRollerSet(0.5),
            new WaitCommand(0.5),
            new ClawRollerSet(0.1)),

        // put the arm/shoulder in
        new SequentialCommandGroup(
            new ShoulderGoto(Shoulder.CUBE_STOW),
            new ShoulderWaitForPosition(),
            new ArmSystemGoTo(Arm.ALMOST_STOW)),
        // drive to the charging station
        new SequentialCommandGroup(
            new DriveBrakeMode(true),
            new DriveStraightOnHeading(-.05, -.5, 25, 130.0 * color),
            new DriveStraightOnHeading(-.5, 40, 130.0 * color),
            new DriveStraightOnHeading(-.5, -.2, 20, 130.0 * color),

            new DriveStraightOnHeading(-.2, -.15, 25, 180.0 * color),
            new DriveStraightOnHeading(-.15, 60, 180.0 * color))));
  }
}
