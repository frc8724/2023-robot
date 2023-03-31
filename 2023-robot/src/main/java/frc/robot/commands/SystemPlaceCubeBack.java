// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmBrake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.ArmBrake.State;

public class SystemPlaceCubeBack extends SequentialCommandGroup {
  /** Creates a new SystemPlaceCubeBack. */
  public SystemPlaceCubeBack(int level) {
    addCommands(
        // move the shoulder and the arm. The shoulder will cause the race to end
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new ShoulderGoto(Shoulder.BACK_ISH),
                new ShoulderWaitForPosition()),
            new SequentialCommandGroup(
                new ArmSystemGoTo(Arm.ALMOST_STOW)
            // new ArmBrakeSet(State.OPEN),
            // new WaitCommand(ArmBrake.OPEN_TIME_SEC),
            // new ArmGoto(Arm.ALMOST_STOW)
            // new WaitCommand(5.0)
            )),
        // new ArmSetPower(0.0, true),
        // new ArmBrakeSet(State.CLOSE),

        // move the should to straight up
        // new ShoulderGoto(Shoulder.STRAIGHT_UP),
        // new ShoulderWaitForPosition(),

        // move teh shoulder and arm to score
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ShoulderGoto(Shoulder.LEVEL_X_PRESCORE[level]),
                new ShoulderWaitForPosition()),
            new SequentialCommandGroup(
                new ArmSystemGoTo(Arm.LEVEL_X_SCORE_Cube[level]))),

        new ClawRollerSet(-.3));

  }
}
