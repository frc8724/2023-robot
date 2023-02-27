// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawPiston.State;
import frc.robot.subsystems.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemScoreAndStow extends SequentialCommandGroup {
  /** Creates a new SystemScoreAndStow. */
  public SystemScoreAndStow() {
    // open the claw
    addCommands(new ClawPistonSet(State.OPEN));
    addCommands(new WaitCommand(0.2));
    // retract the arm
    addCommands(new ArmSystemGoTo(Arm.STOW));
    addCommands(new ArmWaitForPosition());
    // close the claw
    addCommands(new ClawPistonSet(State.CLOSE));
    // lower the shoulder
    addCommands(new ShoulderGoto(Shoulder.STOW));
    addCommands(new ShoulderWaitForPosition());

  }
}
