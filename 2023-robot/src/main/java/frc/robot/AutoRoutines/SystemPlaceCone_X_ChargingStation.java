// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawDropCone;
import frc.robot.commands.DriveSystemOnChargingStation;
import frc.robot.commands.ShoulderGoto;
import frc.robot.commands.ShoulderWaitForPosition;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.commands.SystemStowArm;
import frc.robot.commands.SystemZero;
import frc.robot.subsystems.Shoulder;

public class SystemPlaceCone_X_ChargingStation extends SequentialCommandGroup {
  /** Creates a new Week0_PlaceCone3_ChargingStation. */
  public SystemPlaceCone_X_ChargingStation(int level) {
    addCommands(new SystemZero());

    addCommands(new SystemPlaceCone(level));

    addCommands(new ClawDropCone());

    addCommands(new ShoulderGoto(Shoulder.LEVEL_X_PRESCORE[level]));
    addCommands(new ShoulderWaitForPosition());

    addCommands(new ParallelCommandGroup(
        new SystemStowArm(),
        new DriveSystemOnChargingStation()));
  }
}
