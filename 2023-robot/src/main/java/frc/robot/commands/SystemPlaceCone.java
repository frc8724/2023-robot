// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;

public class SystemPlaceCone extends SequentialCommandGroup {
    /** Creates a new SystemPlaceCone. */
    public SystemPlaceCone(int level) {
        // if the Arm is Further out, retract
        addCommands(new SelectCommand(
                Map.ofEntries(
                        Map.entry(true, new ArmSystemGoTo(Arm.ALMOST_STOW)),
                        Map.entry(false, new WaitCommand(0.0))),
                () -> RobotContainer.arm.getCurrentPosition() > Arm.FLOOR_PICKUP + 1000));

        addCommands(new ParallelCommandGroup(
                new SequentialCommandGroup(
                        // rotate shoulder to the location
                        new ShoulderGoto(Shoulder.LEVEL_X_PRESCORE[level]),
                        new ShoulderWaitForPosition()),

                new SequentialCommandGroup(
                        new SequentialCommandGroup(
                                // wait for the shoulder to get close. Floor pickup is the tolerance distance
                                new ShoulderWaitForPosition(Shoulder.SCORE_TOLERANCE),
                                // extend the arm out
                                new ArmSystemGoTo(Arm.LEVEL_X_SCORE[level]),
                                new ArmWaitForPosition()))));
    }
}
