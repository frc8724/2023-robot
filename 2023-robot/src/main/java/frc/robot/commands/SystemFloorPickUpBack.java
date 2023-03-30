// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.ArmBrake.State;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemFloorPickUpBack extends SequentialCommandGroup {
  /** Creates a new SystemFloorPickUpBack. */
  public SystemFloorPickUpBack() {

    addCommands(new ArmSystemGoTo(Arm.ALMOST_STOW));

    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                // rotate shoulder
                new ShoulderGoto(Shoulder.FLOOR_PICKUP_BACK),
                new ShoulderWaitForPosition()),

            new SequentialCommandGroup(
                // open claw and start sucking
                new WaitCommand(1),
                new ClawPistonSet(ClawPiston.State.OPEN),
                new ClawRollerSet(0.25)),

            new SequentialCommandGroup(
                // extend arm
                new ShoulderWaitForPosition(100000),
                new ArmSystemGoTo(Arm.FLOOR_PICKUP_BACK),
                new ArmWaitForPosition(),
            new ArmSetPower(0.0, true),
            new ArmBrakeSet(State.CLOSE))));
  }
}
