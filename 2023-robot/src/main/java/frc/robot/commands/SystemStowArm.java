// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawPiston.State;
import frc.robot.subsystems.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemStowArm extends SequentialCommandGroup {
  /** Creates a new SystemScoreAndStow. */
  public SystemStowArm() {
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                // retract the arm
                new ArmSystemGoTo(Arm.ALMOST_STOW),
                new ArmWaitForPosition(),
                new ArmSystemZero()),
            new SequentialCommandGroup(
                // new ArmWaitForPosition(Arm.ALMOST_STOW),
                new WaitCommand(0.25),
                new ArmWaitForPosition(450),

                // lower the shoulder
                new ShoulderGoto(Shoulder.STOW),
                new ShoulderWaitForPosition())));
  }
}
