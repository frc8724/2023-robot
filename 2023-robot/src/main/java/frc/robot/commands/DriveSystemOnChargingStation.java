// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Whacker.State;

public class DriveSystemOnChargingStation extends SequentialCommandGroup {
  /** Creates a new DriveSystemOnChargingStation. */
  public DriveSystemOnChargingStation() {
    addCommands(new WhackerSet(State.DOWN));
    addCommands(new WaitCommand(1.0));
    addCommands(new DriveStraightOnHeading(-0.05, -0.1, 50.0, 0));
    addCommands(new WhackerSet(State.UP));
    addCommands(new DriveBrakeMode(true));
  }
}
