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
public class SystemStowArm extends SequentialCommandGroup {
  /** Creates a new SystemScoreAndStow. */
  public SystemStowArm() {
    // retract the arm
    addCommands(new ArmSystemGoTo(Arm.ALMOST_STOW));
    addCommands(new ArmWaitForPosition());
    addCommands(new ArmSystemZero());
    // close the claw
    addCommands(new ClawPistonSet(State.CLOSE));
    // lower the shoulder
    addCommands(new ShoulderGoto(Shoulder.STOW));
    addCommands(new ShoulderWaitForPosition());
  }
}
