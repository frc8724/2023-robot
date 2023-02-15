// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.ClawPiston.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemPlaceCone extends SequentialCommandGroup {
  /** Creates a new SystemPlaceCone. */
  public SystemPlaceCone(int level) {
    // rotate shoulder to the location
    addCommands(new ShoulderGoto(Shoulder.LEVEL_X_PRESCORE[level]));
    addCommands(new ShoulderWaitForPosition());
    // extend the arm out
    addCommands(new ArmGoto(Arm.LEVEL_X_SCORE[level]));
    addCommands(new ArmWaitForPosition());
    // rotate shoulder down to place
    addCommands(new ShoulderGoto(Shoulder.LEVEL_X_SCORE[level]));
    addCommands(new ShoulderWaitForPosition());
    // open the claw
    addCommands(new ClawPistonSet(State.OPEN));
    addCommands(new WaitCommand(0.5));
    // retract the arm
    addCommands(new ArmGoto(Arm.LEVEL_X_SCORE[0]));
    addCommands(new ArmWaitForPosition());
    // lower the shoulder
    addCommands(new ShoulderGoto(Shoulder.LEVEL_X_SCORE[0]));
    addCommands(new ShoulderWaitForPosition());
  }
}
