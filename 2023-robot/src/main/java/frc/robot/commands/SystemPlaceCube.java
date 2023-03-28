// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.ArmBrake.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemPlaceCube extends SequentialCommandGroup {
  /** Creates a new SystemPlaceCube. */
  public SystemPlaceCube(int level) {
    // rotate shoulder to the location
    addCommands(new ShoulderGoto(Shoulder.LEVEL_X_SCORE_CUBE[level]));
    addCommands(new ShoulderWaitForPosition());
    // extend the arm out
    addCommands(new ArmSystemGoTo(Arm.LEVEL_X_SCORE_Cube[level]));
    addCommands(new ParallelRaceGroup(
        new SequentialCommandGroup(
            new ArmWaitForPosition(),
            new ArmSetPower(0.0, true),
            new ArmBrakeSet(State.CLOSE)),

        new SequentialCommandGroup(
            new WaitCommand(0.25),
            new ClawRollerSet(-.5))));
  }
}
