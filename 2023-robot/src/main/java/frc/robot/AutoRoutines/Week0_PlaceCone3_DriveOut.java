// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.DriveToAprilTagDistance;
import frc.robot.commands.SystemPlaceCone;

public class Week0_PlaceCone3_DriveOut extends SequentialCommandGroup {
  /** Creates a new Week0_PlaceCone3_DriveOut. */
  public Week0_PlaceCone3_DriveOut() {
    addCommands(new SystemPlaceCone(3));
    addCommands(new DriveStraightOnHeading(-0.2, 100, 0.0));
    addCommands(new DriveToAprilTagDistance(-0.2, Constants.ConvertFeetToMeters(10.0)));
  }
}
