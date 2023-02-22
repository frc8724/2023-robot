// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShoulderSetPower;
import frc.robot.commands.SystemZero;

public class Week0_StandStill extends SequentialCommandGroup {
  /** Creates a new Week0_StandStill. */
  public Week0_StandStill() {
    addCommands(new SystemZero());
    // addCommands(new ShoulderSetPower(0.10));
  }
}
