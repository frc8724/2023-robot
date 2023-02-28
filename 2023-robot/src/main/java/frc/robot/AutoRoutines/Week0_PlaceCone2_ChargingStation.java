// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveSystemOnChargingStation;
import frc.robot.commands.SystemPlaceCone;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Week0_PlaceCone2_ChargingStation extends SequentialCommandGroup {
  /** Creates a new Week0_PlaceCone2_ChargingStation. */
  public Week0_PlaceCone2_ChargingStation() {
    addCommands(new SystemPlaceCone(2));
    addCommands(new DriveSystemOnChargingStation());
  }
}
