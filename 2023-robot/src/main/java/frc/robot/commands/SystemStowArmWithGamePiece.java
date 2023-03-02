// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawPiston;
import frc.robot.subsystems.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemStowArmWithGamePiece extends SequentialCommandGroup {
  public SystemStowArmWithGamePiece() {
    // close the claw on a cone. Leave open on cube.
    new ClawColorCommand(
        new ClawPistonSet(ClawPiston.State.OPEN),
        new ClawPistonSet(ClawPiston.State.CLOSE));

    // stop the claw
    addCommands(new ClawRollerSet(0.0));

    // retract arm
    addCommands(new ArmSystemGoTo(Arm.ALMOST_STOW));
    addCommands(new ArmWaitForPosition());

    // rotate shoulder
    addCommands(new ShoulderGoto(Shoulder.STOW));
    addCommands(new ShoulderWaitForPosition());
  }
}
