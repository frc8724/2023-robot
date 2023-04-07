// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmGoto;
import frc.robot.commands.ArmSystemGoTo;
import frc.robot.commands.ClawDropCone;
import frc.robot.commands.ClawRollerSet;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.SystemFloorPickUpBack;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.commands.SystemPlaceCubeBack;
import frc.robot.commands.SystemStowArm;
import frc.robot.commands.SystemZero;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawRollers;

public class NECMP_Score2BackBump_Level_X_Color_X extends SequentialCommandGroup {

    public NECMP_Score2BackBump_Level_X_Color_X(int level, int color) {
        addCommands(new SystemZero());
        addCommands(new SystemPlaceCone(level));
        addCommands(new ClawDropCone());

        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                // Start Driving
                                new DriveStraightOnHeading(-0.05, -0.4, 15, 0 * color), // Speed up
                                new DriveStraightOnHeading(-0.4, 33, 0 * color), // Go straight
                                new DriveStraightOnHeading(-0.4, -0.15, 10, 0 * color), // Slow down
                                new DriveStraightOnHeading(-0.15, 21, 5 * color)), // Go over bump
                        new ArmSystemGoTo(Arm.ALMOST_STOW)));// Arm comes in
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                // Go to cube
                                new DriveStraightOnHeading(-0.15, -0.35, 21, 8 * color), // Speed up
                                new DriveStraightOnHeading(-0.35, 95, 8 * color)), // Go straight
                        new SystemFloorPickUpBack()));// Arm flip over
        addCommands(
                new DriveStraightOnHeading(-0.35, -0.1, 15, 8 * color),
                new ClawRollerSet(ClawRollers.SUCK_IN),
                new DriveStraightOnHeading(-0.1, 5, 8 * color));

        addCommands(new DriveStraightOnHeading(0.05, 0.4, 20, 0 * color));
        addCommands(new DriveStraightOnHeading(0.4, 65, 0 * color));
        addCommands(new DriveStraightOnHeading(0.4, 0.15, 15, 0 * color));
        addCommands(new DriveStraightOnHeading(0.15, 15, 0 * color));

        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DriveStraightOnHeading(0.15, 0.4, 10, -10 * color),
                                new DriveStraightOnHeading(0.4, 48, -10 * color),
                                new DriveStraightOnHeading(0.4, 0.05, 25, 0 * color)),
                        new SystemPlaceCubeBack(level)));
        addCommands(new ClawRollerSet(ClawRollers.SPIT));
        addCommands(new WaitCommand(0.2));
        addCommands(new SystemStowArm());

    }
}
