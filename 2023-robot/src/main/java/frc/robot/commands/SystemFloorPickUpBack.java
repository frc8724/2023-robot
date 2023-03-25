// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    addCommands(new ArmWaitForPosition());

    // rotate shoulder
    addCommands(new ShoulderGoto(Shoulder.FLOOR_PICKUP_BACK));
    addCommands(new ShoulderWaitForPosition());

    // open claw and start sucking
    addCommands(new ClawPistonSet(ClawPiston.State.OPEN));
    addCommands(new ClawRollerSet(0.25));

    // extend arm
    addCommands(new ArmSystemGoTo(Arm.FLOOR_PICKUP_BACK));
    addCommands(new ArmWaitForPosition());
    addCommands(new ArmSetPower(0.0, true));
    addCommands(new ArmBrakeSet(State.CLOSE));
  }
}
