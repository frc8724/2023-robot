// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmSystemGoTo;
import frc.robot.commands.ArmZero;
import frc.robot.commands.ClawPistonSet;
import frc.robot.commands.ClawRollerSet;
import frc.robot.commands.DriveBrakeMode;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.ShoulderGoto;
import frc.robot.commands.ShoulderWaitForPosition;
import frc.robot.commands.SystemZero;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.ClawPiston.State;

public class Week3_PlaceCone_Charging_Level_X_Color_X extends SequentialCommandGroup {
  public Week3_PlaceCone_Charging_Level_X_Color_X(int Level, int color) {
    addCommands(new SystemZero());

    addCommands(new Week3_PlaceCone_GrabAnother_Level_X_Color_X(Level, color));

    addCommands(new DriveStraightOnHeading(.1, .2, 15, 80 * color));

    addCommands(new ClawPistonSet(State.CLOSE));

    addCommands(new ParallelCommandGroup(
        // stutter the claw rollers to settle the
        new SequentialCommandGroup(
            new WaitCommand(1.0),
            new ClawRollerSet(0.2),
            new WaitCommand(1.0),
            new ClawRollerSet(0.5),
            new WaitCommand(0.5),
            new ClawRollerSet(0.1)),
        // put the arm/shoulder in
        new SequentialCommandGroup(
            new ArmSystemGoTo(Arm.ALMOST_STOW),
            // new ArmZero(),
            new ShoulderGoto(Shoulder.CONE_STOW),
            new ShoulderWaitForPosition()),
        // drive to the charging station
        new SequentialCommandGroup(
            new DriveStraightOnHeading(.1, .4, 10, 45.0 * color),
            new DriveStraightOnHeading(.4, 55, 45.0 * color),
            new DriveBrakeMode(true),
            new DriveStraightOnHeading(.4, 30, 0.0 * color),

            new DriveStraightOnHeading(.4, .15, 10, 0.0 * color),
            new DriveBrakeMode(true),
            new DriveStraightOnHeading(.15, .2, 45, 0.0 * color),
            new DriveBrakeMode(true),
            new DriveStraightOnHeading(.2, .1, 25, 0.0 * color),
            new DriveBrakeMode(true))));

    // addCommands(new DriveStraightOnHeading(.2, 50, 0.0 * color));
  }
}
