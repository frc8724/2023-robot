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
    addCommands(new WaitCommand(4.0));

    addCommands(new DriveStraightOnHeading(-0.05, -0.12, 20.0, 0));
    addCommands(new DriveStraightOnHeading(-0.12, 46.0, 0));
    addCommands(new WhackerSet(State.UP));
    addCommands(new DriveStraightOnHeading(-0.15, -0.1, 20.0, 0));
    // addCommands(new DriveStraightOnHeading(-0.1, -0.05, 7.0, 45));

    addCommands(new DriveBrakeMode(true));
    // addCommands(new DriveStraightOnHeading(-0.15, -0.05, 22.0, 0));
    // addCommands(new DriveStraightOnHeading(-0.15, -0.05, 22.0, 0));

  }
}
