// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.ClawPiston.State;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemFloorPickUp extends SequentialCommandGroup {
  /** Creates a new SystemFloorPickUp. */
  public SystemFloorPickUp() {

    // move shoulder
    addCommands(new ShoulderGoto(Shoulder.FLOOR_PICKUP));
    addCommands(new ShoulderWaitForPosition());
    // claw open
    addCommands(new ClawPistonSet(State.OPEN));
    addCommands(new ClawRollerSet(0.5));

    // extend arm
    addCommands(new ArmSystemGoTo(Arm.FLOOR_PICKUP));
    addCommands(new ArmWaitForPosition());
    addCommands(new ArmSetPower(0.0, true));
    addCommands(new ArmBrakeSet(ArmBrake.State.CLOSE));

  }
}
