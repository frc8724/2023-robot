// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmBrake.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmSystemGoTo extends SequentialCommandGroup {
  /** Creates a new ArmSystemGoTo. */
  public ArmSystemGoTo(double pos) {
    addCommands(new ArmBrakeSet(State.OPEN));
    addCommands(new WaitCommand(0.25));
    addCommands(new ArmGoto(pos));
    addCommands(new ArmWaitForPosition());
    addCommands(new WaitCommand(0.25));
    addCommands(new ArmBrakeSet(State.CLOSE));
  }
}
